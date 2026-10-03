package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * The four-wheel mecanum drive (goBILDA GripForce 104 mm wheels, 5203-2402-0019 motors, 312 RPM).
 * Mount the wheels so the rollers form an "X" when you look down on the robot from above.
 *
 * TeleOp: drive(forward, strafe, rotate) every loop, or driveFieldCentric(...), which uses the
 *         Control Hub's built-in IMU so "stick up" always means "away from the driver".
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
    }

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
     * Drive with three "powers" from -1 to 1: forward (+ = forward), strafe (+ = right),
     * rotate (+ = turn right). This is the mecanum formula.
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
