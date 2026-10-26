package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * The four-wheel mecanum drive (goBILDA GripForce 104 mm wheels, 5203-2402-0019 motors, 312 RPM).
 * Mount the wheels so the rollers form an "X" when you look down on the robot from above.
 *
 * TeleOp: driveWithSticks(driver) every loop drives with the driver's sticks, with the driver-feel
 *         numbers below (deadband, slow mode, smooth speed-up). It is ROBOT-CENTRIC: "stick up" means
 *         the way the robot's front faces (the TeleOp's default). Field-centric ("stick up" = away
 *         from the driver, with the Control Hub's built-in IMU) only when RobotSwitches.USE_FIELD_CENTRIC.
 *         driveToward(...) is for auto-align. drive(forward, strafe, rotate) is the plain mecanum formula.
 * Auto:   driveDistance(), strafeDistance(), rotateAngle() return true when the move is done;
 *         call resetEncoders() between moves so each one counts from 0.
 *
 * Each wheel uses velocity control: a "power" from -1 to 1 becomes a fraction of
 * MAX_TICKS_PER_SEC, and the motor's PIDF control holds that speed on a tired battery too.
 * Tune PIDF in FTC Dashboard under "MecanumDriveSubsystem" while driving: a changed number reaches the
 * wheels within one loop. Graph each wheel's "velocity" against its "target". When it looks good, copy
 * the numbers into PIDF below, because Dashboard changes are lost when the robot restarts.
 */
@Config
public class MecanumDriveSubsystem {
    // ---- Tuning numbers (owner: drive team) ----
    // F = 32767 / 2800 = 11.7,  P = 0.1 * F,  I = 0.1 * P,  D = 0
    public static PIDFCoefficients PIDF = new PIDFCoefficients(1.17, 0.117, 0, 11.7);   // p, i, d, f
    // A bit below the ~2800 ticks/s top speed, so PIDF always has power left to hold the speed.
    public static double MAX_TICKS_PER_SEC = 2500;

    // ---- Robot measurements, used by the Auto's distance moves ----
    public static double WHEEL_DIAMETER_MM = 104;    // goBILDA GripForce mecanum wheel
    public static double TICKS_PER_REV = 537.7;      // 5203-2402-0019: 28 * 19.2
    // Distance between the left and right wheel centers. Mecanum rollers slip a little while
    // turning, so test rotateAngle() and adjust until a 90 degree request turns 90 degrees.
    public static double TRACK_WIDTH_MM = 402;
    // Strafing slips more than driving. Ask for 500 mm, measure, set this to 500 / measured.
    public static double STRAFE_MULTIPLIER = 1.0;
    // A move counts as done when the front-left wheel is this close to its target.
    public static double TOLERANCE_MM = 10;

    // ---- Auto: driving to a spot with odometry (driveToPoint) and turning (turnToHeading) ----
    // driveToPoint() drives straight at the spot, slower as it gets close: GOTO_GAIN power per inch
    // still to go, never below GOTO_MIN_POWER (enough to keep moving when close) or above GOTO_MAX_POWER.
    public static double GOTO_TOLERANCE_IN = 1.5;    // "there" when this close
    public static double GOTO_MAX_POWER = 0.4;
    public static double GOTO_MIN_POWER = 0.1;
    public static double GOTO_GAIN = 0.05;
    public static double GOTO_HEADING_GAIN = 0.01;   // turn power per degree, to keep the heading
    // turnToHeading() is "there" when the heading is this close.
    public static double HEADING_TOLERANCE_DEG = 3;

    // ---- Driver feel, TeleOp only (owner: drive team) ----
    // Stick values smaller than this count as 0, so a stick that doesn't center perfectly
    // doesn't make the robot creep.
    public static double DEADBAND = 0.05;
    // Slow mode multiplies every drive command by this (0.4 = 40% speed).
    public static double SLOW_MODE_SCALE = 0.4;
    // How fast a drive command may change, per second. 4 means 0 to full speed in 0.25 s:
    // smooth enough that the robot doesn't jerk or tip, quick enough to feel direct.
    public static double MAX_ACCEL_PER_SEC = 4.0;
    // false: robot-centric, stick up = the way the robot's front faces (the team's choice).
    // true: start in field-centric mode when the IMU is found (stick up = away from the driver).
    // Only used when RobotSwitches.USE_FIELD_CENTRIC is true.
    public static boolean FIELD_CENTRIC_AT_START = false;

    /*
     * How the Control Hub is mounted, for the built-in IMU (field-centric driving, and the Auto's
     * SEARCH and PARK). Look at the hub: which way does the REV logo face, and which way do the USB
     * ports point? The two must be at right angles.
     *   Flat on the chassis, logo up:            HUB_LOGO = UP,    HUB_USB = the way the ports point
     *   Upright on the robot's LEFT side, logo out:  HUB_LOGO = LEFT,  HUB_USB = UP, DOWN, FORWARD or BACKWARD
     *   Upright on the robot's RIGHT side, logo out: HUB_LOGO = RIGHT, HUB_USB = UP, DOWN, FORWARD or BACKWARD
     * Check: in the Auto's INIT, turn the robot left by hand; "Heading" must go up. If it goes down
     * or barely moves, these two are wrong. roadrunner/MecanumDrive PARAMS needs the same values.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx[] motors;
    private final ElapsedTime holdTimer = new ElapsedTime();
    // The PIDF numbers the wheels have now (NaN = never sent).
    private final PIDFCoefficients sentPidf = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    // The Control Hub's built-in IMU, or null if "imu" is not in the configuration.
    private final IMU imu;

    // The speed (ticks per second) each wheel was last asked for, for telemetry.
    private double leftFrontTarget, rightFrontTarget, leftBackTarget, rightBackTarget;

    /*
     * ODOMETRY: where the robot is, counted from where it was at INIT.
     * x = inches forward (the way the robot faced at INIT), y = inches to the left, heading in degrees
     * (+ = turned left). Wheels slip a little, so this drifts over a long match.
     */
    private double poseX = 0, poseY = 0, wheelHeadingRad = 0;
    private final int[] lastTicks = new int[4];
    // Field-centric "forward" is set by resetHeading(); odometry keeps its own heading.
    private double headingOffsetDeg = 0;

    // TeleOp: the modes the driver switches, the smoothed drive commands, and the time between them.
    private boolean slowMode = false;
    private boolean fieldCentric = false;
    private double forwardCmd = 0, strafeCmd = 0, rotateCmd = 0;
    private final ElapsedTime smoothTimer = new ElapsedTime();
    // Auto: true when the last driveToPoint() was stopped from driving further forward.
    private boolean forwardLimited = false;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        leftBack = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        motors = new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};

        /*
         * The motors on the left side face the other way, so they are reversed. Pushing the left
         * stick forward MUST make the robot go forward; change these after your first test drive.
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(BRAKE);
        }
        resetEncoders();
        // The Control Hub runs the PIDF loop itself; we only give each wheel the numbers.
        sendPidfIfChanged();

        /*
         * The IMU tells us which way the robot faces. tryGet() returns null instead of crashing
         * when "imu" is missing from the configuration; then field-centric driving falls back to
         * normal (robot-centric) driving.
         */
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
        }
        fieldCentric = RobotSwitches.USE_FIELD_CENTRIC && FIELD_CENTRIC_AT_START && imu != null;
    }

    // ---------------- TeleOp driving ----------------

    // Call at START, so the first smooth step counts from START and not from INIT.
    public void startTeleop() {
        smoothTimer.reset();
    }

    // Driver X: slow mode on/off (for lining up).
    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    // Driver Y: field-centric on/off. Does nothing unless RobotSwitches.USE_FIELD_CENTRIC is true
    // and the IMU was found.
    public void toggleFieldCentric() {
        if (RobotSwitches.USE_FIELD_CENTRIC && imu != null) {
            fieldCentric = !fieldCentric;
        }
    }

    // How the robot drives now, for telemetry: "robot-centric, SLOW" and so on.
    public String getModeText() {
        String mode = fieldCentric ? "field-centric" : (imu != null ? "robot-centric" : "robot-centric (no IMU)");
        return mode + (slowMode ? ", SLOW" : "");
    }

    // Call once per loop: left stick drives and strafes, right stick X turns.
    public void driveWithSticks(Gamepad driver) {
        driveWithSticks(driver, deadband(driver.right_stick_x));
    }

    /*
     * The same, but the robot turns with "turn" instead of the right stick (for example to aim at
     * the CELL). Moving the left stick forward gives a negative number on most gamepads, so we flip it.
     */
    public void driveWithSticks(Gamepad driver, double turn) {
        double forward = deadband(-driver.left_stick_y);
        double strafe = deadband(driver.left_stick_x);
        double scale = slowMode ? SLOW_MODE_SCALE : 1;
        driveSmoothly(forward * scale, strafe * scale, turn * scale, fieldCentric);
    }

    // Call once per loop instead of driveWithSticks() for commands that don't come from the sticks
    // (auto-align): always robot-centric, never slowed down.
    public void driveToward(double forward, double strafe, double rotate) {
        driveSmoothly(forward, strafe, rotate, false);
    }

    // Each command moves toward what was asked by at most MAX_ACCEL_PER_SEC per second, so the robot
    // speeds up and slows down gently. A slow loop counts as at most 0.1 s, so it can't cause a jump.
    private void driveSmoothly(double forward, double strafe, double rotate, boolean useFieldCentric) {
        double dt = Math.min(smoothTimer.seconds(), 0.1);
        smoothTimer.reset();
        double step = MAX_ACCEL_PER_SEC * dt;
        forwardCmd = approach(forwardCmd, forward, step);
        strafeCmd = approach(strafeCmd, strafe, step);
        rotateCmd = approach(rotateCmd, rotate, step);
        if (useFieldCentric) {
            driveFieldCentric(forwardCmd, strafeCmd, rotateCmd);
        } else {
            drive(forwardCmd, strafeCmd, rotateCmd);
        }
    }

    // Stick values smaller than DEADBAND count as 0.
    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }

    // Move "current" toward "target" by at most "step".
    private static double approach(double current, double target, double step) {
        return current + Range.clip(target - current, -step, step);
    }

    // ---------------- Auto: drive to a spot, turn to a heading, turn to a CELL ----------------

    /*
     * One loop of driving straight at a spot, measured from where the robot stood at INIT (inches
     * forward, inches to the left), while holding headingDeg. Every loop it asks odometry where the
     * robot really is, so it does not matter how it got there. It never drives further forward than
     * maxForwardIn (it may still slide sideways or back up); hitForwardLimit() says when that held it back.
     * Returns true when the robot is within GOTO_TOLERANCE_IN of the spot.
     */
    public boolean driveToPoint(double forwardIn, double leftIn, double headingDeg, double maxForwardIn) {
        // How far the spot still is, in the INIT directions (x forward, y left)...
        double dx = forwardIn - poseX;
        double dy = leftIn - poseY;
        forwardLimited = poseX >= maxForwardIn && dx > 0;
        if (forwardLimited) {
            dx = 0;
        }
        double distance = Math.hypot(dx, dy);
        if (distance < GOTO_TOLERANCE_IN) {
            stop();
            return true;
        }
        // ...turned into the robot's own directions, using the heading it has now.
        double h = Math.toRadians(getPoseHeadingDeg());
        double ahead = dx * Math.cos(h) + dy * Math.sin(h);
        double toLeft = -dx * Math.sin(h) + dy * Math.cos(h);
        double power = Range.clip(GOTO_GAIN * distance, GOTO_MIN_POWER, GOTO_MAX_POWER);
        double headingError = headingDeg - getPoseHeadingDeg();
        // In drive(), + strafe goes right and + rotate turns right, so "left" needs a minus sign.
        drive(power * ahead / distance, -power * toLeft / distance, -GOTO_HEADING_GAIN * headingError);
        return false;
    }

    // True when the last driveToPoint() was held back by its maxForwardIn.
    public boolean hitForwardLimit() {
        return forwardLimited;
    }

    /*
     * One loop of turning in place at "power" toward a heading (degrees since INIT, + = left).
     * Returns true when the robot is within HEADING_TOLERANCE_DEG of it.
     */
    public boolean turnToHeading(double targetDeg, double power) {
        double error = AngleUnit.normalizeDegrees(targetDeg - getPoseHeadingDeg());
        if (Math.abs(error) < HEADING_TOLERANCE_DEG) {
            stop();
            return true;
        }
        // + error means turn left, but in drive() a + rotate turns right, so we flip the sign.
        drive(0, 0, error > 0 ? -power : power);
        return false;
    }

    /*
     * One loop of turning in place so the LAUNCHER points at the CELL the camera sees (the same rule
     * as the TeleOp's left bumper). Returns true when it does (within AprilTagVision.AIM_TOLERANCE_DEG).
     */
    public boolean turnToCell(AprilTagVision vision, AprilTagDetection cell) {
        if (vision.isAimedAt(cell)) {
            stop();
            return true;
        }
        drive(0, 0, vision.aimTurn(vision.launcherBearing(cell)));
        return false;
    }

    // ---------------- Heading and odometry ----------------

    // True when the IMU was found, so field-centric driving works.
    public boolean hasImu() {
        return imu != null;
    }

    // Field-centric heading in degrees: + = turned left since the last resetHeading().
    public double getHeadingDegrees() {
        return AngleUnit.normalizeDegrees(getPoseHeadingDeg() - headingOffsetDeg);
    }

    /*
     * "The way the robot faces now is field forward." Press this with the robot facing away from you.
     * Only field-centric driving uses it; the odometry pose keeps counting from INIT.
     */
    public void resetHeading() {
        headingOffsetDeg = getPoseHeadingDeg();
    }

    // Odometry pose: inches forward and left of where the robot was at INIT, heading in degrees.
    public double getPoseX() {
        return poseX;
    }

    public double getPoseY() {
        return poseY;
    }

    // Heading since INIT: from the IMU, or from the wheels when there is no IMU.
    public double getPoseHeadingDeg() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return Math.toDegrees(wheelHeadingRad);
    }

    /*
     * Add how far each wheel turned since the last loop to the pose (mecanum forward kinematics):
     *   forward = average of all four wheels,  right = (FL - FR - BL + BR) / 4,
     *   turn    = (-FL + FR - BL + BR) / 4 / half the track width.
     * Then turn that robot move into field x and y using the heading.
     */
    private void updatePose() {
        int[] now = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(), rightBack.getCurrentPosition()};
        double ticksPerInch = ticksPerMm() * 25.4;
        double fl = (now[0] - lastTicks[0]) / ticksPerInch, fr = (now[1] - lastTicks[1]) / ticksPerInch;
        double bl = (now[2] - lastTicks[2]) / ticksPerInch, br = (now[3] - lastTicks[3]) / ticksPerInch;
        System.arraycopy(now, 0, lastTicks, 0, 4);

        double forward = (fl + fr + bl + br) / 4;
        double left = -(fl - fr - bl + br) / 4 / STRAFE_MULTIPLIER;
        wheelHeadingRad += (-fl + fr - bl + br) / 4 / (TRACK_WIDTH_MM / 25.4 / 2);

        double h = Math.toRadians(getPoseHeadingDeg());
        poseX += forward * Math.cos(h) - left * Math.sin(h);
        poseY += forward * Math.sin(h) + left * Math.cos(h);
    }

    /*
     * Field-centric driving: forward (+ = away from the driver) and strafe (+ = to the driver's
     * right) are in field directions, whichever way the robot faces. We turn the stick direction
     * by minus the robot's heading, then drive as usual. Without an IMU this is the same as drive().
     */
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        if (imu == null) {
            drive(forward, strafe, rotate);
            return;
        }
        double heading = Math.toRadians(getHeadingDegrees());
        double robotStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double robotForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        drive(robotForward, robotStrafe, rotate);
    }

    // Distance the robot moves for one encoder tick: 537.7 / (pi * 104) = 1.65 ticks per mm.
    public static double ticksPerMm() {
        return TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    }

    // Call once per loop: updates the odometry pose.
    public void update() {
        sendPidfIfChanged();
        updatePose();
    }

    // Sends PIDF to the four wheels when a number changed (in FTC Dashboard), not every loop.
    private void sendPidfIfChanged() {
        if (PIDF.p == sentPidf.p && PIDF.i == sentPidf.i && PIDF.d == sentPidf.d && PIDF.f == sentPidf.f) {
            return;
        }
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        }
        sentPidf.p = PIDF.p;
        sentPidf.i = PIDF.i;
        sentPidf.d = PIDF.d;
        sentPidf.f = PIDF.f;
    }

    /*
     * Robot-centric driving with three "powers" from -1 to 1: forward (+ = toward the robot's front),
     * strafe (+ = the robot's right), rotate (+ = turn right). This is the mecanum formula.
     */
    public void drive(double forward, double strafe, double rotate) {
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        // If any wheel is asked for more than 1, slow them all down by the same amount.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setWheelVelocities(leftFrontPower * MAX_TICKS_PER_SEC, rightFrontPower * MAX_TICKS_PER_SEC,
                leftBackPower * MAX_TICKS_PER_SEC, rightBackPower * MAX_TICKS_PER_SEC);
    }

    public void stop() {
        setWheelVelocities(0, 0, 0, 0);
    }

    // Set every wheel's position count back to 0. Call between Auto moves.
    public void resetEncoders() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // The counts are 0 again, so odometry must not see this as a big move.
        java.util.Arrays.fill(lastTicks, 0);
    }

    /**
     * Drives straight. Call every loop until it returns true.
     * @param speed From 0-1
     * @param distance Positive = forward
     * @param holdSeconds how long the robot must stay at the target before this returns true
     */
    public boolean driveDistance(double speed, double distance, DistanceUnit unit, double holdSeconds) {
        double target = unit.toMm(distance) * ticksPerMm();
        // Driving straight on mecanum wheels: all four wheels turn the same way, the same amount.
        runToPositions(speed, target, target, target, target);
        return heldAt(target, holdSeconds);
    }

    /**
     * Slides sideways without turning. Only mecanum wheels can do this!
     * @param distance Positive = slide RIGHT, negative = slide LEFT
     */
    public boolean strafeDistance(double speed, double distance, DistanceUnit unit, double holdSeconds) {
        double target = unit.toMm(distance) * ticksPerMm() * STRAFE_MULTIPLIER;
        // To slide right, front-left and back-right spin forward, front-right and back-left backward.
        runToPositions(speed, target, -target, -target, target);
        return heldAt(target, holdSeconds);
    }

    /**
     * Turns in place.
     * @param angle Positive = turn LEFT
     */
    public boolean rotateAngle(double speed, double angle, AngleUnit unit, double holdSeconds) {
        // One radian of turning = each wheel drives half the track width along its circle.
        double targetMm = unit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double left = -(targetMm * ticksPerMm());
        double right = targetMm * ticksPerMm();
        runToPositions(speed, left, right, left, right);
        return heldAt(left, holdSeconds);
    }

    // Sends each wheel to its own target position, all at the same speed (0-1 of MAX_TICKS_PER_SEC).
    private void runToPositions(double speed, double lf, double rf, double lb, double rb) {
        leftFront.setTargetPosition((int) lf);
        rightFront.setTargetPosition((int) rf);
        leftBack.setTargetPosition((int) lb);
        rightBack.setTargetPosition((int) rb);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(speed * MAX_TICKS_PER_SEC);
        }
    }

    // True once the front-left wheel has stayed within TOLERANCE_MM of its target for holdSeconds.
    private boolean heldAt(double frontLeftTarget, double holdSeconds) {
        if (Math.abs(frontLeftTarget - leftFront.getCurrentPosition()) > TOLERANCE_MM * ticksPerMm()) {
            holdTimer.reset();
        }
        return holdTimer.seconds() > holdSeconds;
    }

    private void setWheelVelocities(double lf, double rf, double lb, double rb) {
        leftFrontTarget = lf;
        rightFrontTarget = rf;
        leftBackTarget = lb;
        rightBackTarget = rb;
        leftFront.setVelocity(lf);
        rightFront.setVelocity(rf);
        leftBack.setVelocity(lb);
        rightBack.setVelocity(rb);
    }

    /*
     * Real speed of each wheel. With the Dashboard on, also the speed each wheel was asked for:
     * graph "front left target" next to "front left velocity" (FTC Dashboard or AdvantageScope)
     * to spot a loose encoder, a reversed encoder or a wheel that rubs.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("front left velocity", leftFront.getVelocity());
        telemetry.addData("front right velocity", rightFront.getVelocity());
        telemetry.addData("back left velocity", leftBack.getVelocity());
        telemetry.addData("back right velocity", rightBack.getVelocity());
        if (RobotSwitches.USE_DASHBOARD) {
            telemetry.addData("front left target", leftFrontTarget);
            telemetry.addData("front right target", rightFrontTarget);
            telemetry.addData("back left target", leftBackTarget);
            telemetry.addData("back right target", rightBackTarget);
        }
    }

    // Wheel positions, for the Auto's distance moves.
    public void addPositionTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor Current Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFront.getTargetPosition(), rightFront.getTargetPosition(),
                leftBack.getTargetPosition(), rightBack.getTargetPosition());
    }
}
