package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class MecanumDrive {
    /*
     * How the Control Hub is mounted, for the built-in IMU (field-centric driving, and the Auto's
     * SEARCH and PARK). Look at the hub: which way does the REV logo face, and which way do the USB
     * ports point? The two must be at right angles.
     *   Flat on the chassis, logo up:            HUB_LOGO = UP,    HUB_USB = the way the ports point
     *   Upright on the robot's LEFT side, logo out:  HUB_LOGO = LEFT,  HUB_USB = UP, DOWN, FORWARD or BACKWARD
     *   Upright on the robot's RIGHT side, logo out: HUB_LOGO = RIGHT, HUB_USB = UP, DOWN, FORWARD or BACKWARD
     * Check: in the Auto's INIT, turn the robot left by hand; "Heading" must go up. If it goes down
     * or barely moves, these two are wrong. roadrunner/MecanumDrive PARAMS needs the same values.
     * Our robot: upright on its side, logo toward the robot's LEFT, USB ports pointing DOWN.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    private double poseX = 0, poseY = 0, wheelHeadingRad = 0;

    private final int[] lastTicks = new int[4];

    private double FL_COMPENSATION=0.0;
    private double      FR_COMPENSATION=0.0;
    private double  BL_COMPENSATION=0.0;
    private double        BR_COMPENSATION=0.0;

    // ---- driveDistance tuning (all need a field check) ----
    // ---------- 2. Add inside MecanumDrive ----------
    public static double STRAFE_MIN_POWER = 0.30;   // strafing needs more power than driving to move cleanly
    public static double CROSS_KP = 0.03;           // creep correction: power per inch of creep
    public static double MAX_CROSS = 0.15;
    public static double PRE_ALIGN_DEG = 2.0;       // start each move square if off by more than this

    public static double HEADING_KP = 0.02;            // turn power per degree of yaw error
    public static double MAX_HEADING_CORRECTION = 0.25;
    public static double MIN_POWER = 0.20;             // lowest power that still moves the robot on the tile
    public static double RAMP_UP_IN = 4;               // inches to reach full speed
    public static double RAMP_DOWN_IN = 8;             // inches over which it slows down at the end
    // Strafing covers less ground per wheel tick than driving. Ask for 24 in, measure, set this to
// 24 / measured (for example robot moved 17 in -> 1.41). 1.0 = same as driving.
    public static double STRAFE_TICKS_FACTOR = 1.0;
    // If a strafe also creeps forward/back, add a little forward power (fraction of the move power;
// + = forward, - = back). 0 = none. Tune per side.
    public static double LEFT_FORWARD_TRIM = 0.0;
    public static double RIGHT_FORWARD_TRIM = 0.0;

    HardwareMap hardwareMap; Telemetry telemetry;
    //OctoQuad octoQuad;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
//
//    DistanceSensor frontDistance;
//    DistanceSensor leftDistance;
//    DistanceSensor rightDistance;
//    DistanceSensor backDistance;

    // This declares the IMU needed to get the current direction the robot is facing
    public IMU imu;

    String flDrive = "front_left_motor";
    String frDrive = "front_right_motor";
    String blDrive = "back_left_motor";
    String brDrive = "back_right_motor";
    private  DcMotor[] motors;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, flDrive);
        frontRightDrive = hardwareMap.get(DcMotor.class, frDrive);
        backLeftDrive = hardwareMap.get(DcMotor.class, blDrive);
        backRightDrive = hardwareMap.get(DcMotor.class, brDrive);

        motors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};

       /* octoQuad = hardwareMap.get(OctoQuad.class, "octoquad");

        /* we don't have a distance sensor so we will take out the code that uses the distance sensor
         frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
         leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
         rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
         backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

         */
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(BRAKE);
        }

        resetEncoders();
        /*
         * The IMU tells us which way the robot faces. tryGet() returns null instead of crashing
         * when "imu" is missing from the configuration; then field-centric driving falls back to
         * normal (robot-centric) driving.
         */
        imu = hardwareMap.get(IMU.class, "imu");

        //imu.initialize(new IMU.Parameters(orientationOnRobot));
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
        }
    }
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // AI Helped
    public void  mecanumDrive(double forward, double strafe, double rotate) {

        // --- Configuration Constants ---
        // Max power robot can reach (e.g., 0.8 for 80% of full speed)
        final double MAX_SPEED_FACTOR = 0.8;
        // Power Smoothing Exponent: 1.0 is linear, 3.0 is cubic (more sensitive to small inputs)
        final double POWER_SMOOTHING_EXPONENT = 3.0;

        // --- 1. Apply Smoothing (Non-Linear Power Curve) ---
        // Uses the formula: sign(x) * |x|^P, which gives finer control at low speeds.
        double f = Math.copySign(Math.pow(Math.abs(forward), POWER_SMOOTHING_EXPONENT), forward);
        double s = Math.copySign(Math.pow(Math.abs(strafe), POWER_SMOOTHING_EXPONENT), strafe);
        double r = Math.copySign(Math.pow(Math.abs(rotate), POWER_SMOOTHING_EXPONENT), rotate);

        // --- 2. Calculate Raw Motor Powers (Mecanum Kinematics) ---
        /* The signs for 's' are crucial. The standard set is (+s, -s, -s, +s).
         * If strafing is reversed, change all 's' signs.
         * e.g., for LeftFront use (f - s + r) instead of (f + s + r)
         */

        double leftFrontRawPower = f + s + r;
        double rightFrontRawPower = f - s - r;
        double leftBackRawPower = f - s + r;
        double rightBackRawPower = f + s - r;


//        // Option B: Use this if strafing is reversed with the standard signs.
//        double leftFrontRawPower = f - s + r; // <-- s sign inverted
//        double rightFrontRawPower = f + s - r; // <-- s sign inverted
//        double leftBackRawPower = f + s + r; // <-- s sign inverted
//        double rightBackRawPower = f - s - r; // <-- s sign inverted

        // --- 3. Normalization (Scaling) ---
        /* Denominator is the largest absolute power required across all motors, or 1.
         * This ensures no power value exceeds 1.0 *before* the MAX_SPEED_FACTOR is applied.
         */
        double maxMagnitude = Math.max(
                Math.abs(leftFrontRawPower),
                Math.max(
                        Math.abs(rightFrontRawPower),
                        Math.max(Math.abs(leftBackRawPower), Math.abs(rightBackRawPower))
                )
        );
        // Use Math.max(maxMagnitude, 1) to prevent division by a number less than 1,
        // which would unnecessarily scale up small powers (though power smoothing makes this less likely)
        double denominator = Math.max(maxMagnitude, 1.0);

        // --- 4. Final Power Calculation and Limiting ---

        // Calculate final power and scale by the MAX_SPEED_FACTOR
        leftFrontPower = (leftFrontRawPower / denominator) * MAX_SPEED_FACTOR;
        rightFrontPower = (rightFrontRawPower / denominator) * MAX_SPEED_FACTOR;
        leftBackPower = (leftBackRawPower / denominator) * MAX_SPEED_FACTOR;
        rightBackPower = (rightBackRawPower / denominator) * MAX_SPEED_FACTOR;

        // --- 5. Set Motor Powers ---
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);



    }

    // Set every wheel's position count back to 0. Call between Auto moves.
    public void resetEncoders() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // The counts are 0 again, so odometry must not see this as a big move.
        java.util.Arrays.fill(lastTicks, 0);
    }

    public double getPoseHeadingDeg() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return Math.toDegrees(wheelHeadingRad);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate


        // 1. Sensitivity Control Constants
        // Adjust these to fine-tune the robot's feel.
        final double MAX_SPEED_FACTOR = 0.8; // Max power robot can reach (e.g., 0.8 for 80%)
        final double POWER_SMOOTHING_EXPONENT = 3.0; // 1.0 is linear, 3.0 is cubic (more sensitive to small inputs)

        // 2. Apply Smoothing (Non-Linear Power Curve)
        // Uses the formula: sign(x) * |x|^P, where P is the exponent.
        // This gives finer control at low speeds.
        double f = Math.copySign(Math.pow(forward, POWER_SMOOTHING_EXPONENT), forward);
        double s = Math.copySign(Math.pow(right, POWER_SMOOTHING_EXPONENT), right);
        double r = Math.copySign(Math.pow(rotate, POWER_SMOOTHING_EXPONENT), rotate);


      //  double frontLeftPower = forward + right + rotate;
    //    double frontRightPower = forward - right - rotate;
      //  double backRightPower = forward + right - rotate;
     //   double backLeftPower = forward - right + rotate;

        double frontLeftPower = f + s + r;
        double frontRightPower = f - s - r;
        double backLeftPower = f - s + r;
        double backRightPower = f + s - r;

        double maxPower = 1.0;
        double maxSpeed = MAX_SPEED_FACTOR; //1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));

        //telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
//            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
        telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);

//        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));

        telemetry.addData("Encoders"," %d %d %d %d", backLeftDrive.getCurrentPosition(), frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
//        telemetry.addData("Octoquad", "%d %d %d %d", octoQuad.readSinglePosition(0),
//                octoQuad.readSinglePosition(1), octoQuad.readSinglePosition(2),
//                octoQuad.readSinglePosition(3));
        telemetry.update();
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    /***************************************Autonomous code*****************************/
    private static final double COUNTS_PER_MOTOR_REV = 537.7;  // eg: GoBILDA 312 RPM
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    //private static final double WHEEL_DIAMETER_INCHES = 4.094;

    // ---------- 1. Change this existing line (it was 4.0) ----------
    private static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;   // 104 mm wheel


    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    private double holdHeading = 0;                 // "straight", in degrees
    // A LinearOpMode can set this to this::opModeIsActive. The default (always true) is safe: the timeout still stops moves.
    public java.util.function.BooleanSupplier isActive = () -> true;

    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    public void setHoldHeading(double deg) { holdHeading = deg; }

    // Call from init_loop() (or a wait loop). Kids square the robot to the wall and press A.
    public void squareUpStep(boolean zeroButton) {
        if (zeroButton) imu.resetYaw();             // robot is square NOW -> 0 degrees
        double yaw = getYaw();
        telemetry.addData("Yaw", "%.1f deg", yaw);
        if (Math.abs(yaw) < 1.0)  telemetry.addLine("SQUARE - read");
        else if (yaw > 0)         telemetry.addLine("Turn the robot CLOCKWISE (right) a little");
        else                      telemetry.addLine("Turn the robolittle");
        telemetry.addLine("Push robot square to the wall, then press A");
        holdHeading = 0;
    }


    private Pose2d robotPose = new Pose2d(0.0, 0.0, 0.0);

    // Inner class for 2D Pose
    private class Pose2d {
        public double x;      // X position in inches
        public double y;      // Y position in inches
        public double heading; // Heading in radians

        public Pose2d(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public double getHeadingDegrees() {
            return Math.toDegrees(heading);
        }

        public void setHeadingDegrees(double degrees) {
            heading = Math.toRadians(degrees);
        }

        public Pose2d copy() {
            return new Pose2d(x, y, heading);
        }

        @Override
        public String toString() {
            return String.format("Pose2d(x=%.2f, y=%.2f, heading=%.1f°)",
                    x, y, getHeadingDegrees());
        }
    }

//    public void driveDistance(String direction, double inches, double speed) {
//        int targetCounts = (int)(inches * COUNTS_PER_INCH);
//        double power = 0.6;
//        // Store starting pose
//        Pose2d startPose = robotPose.copy();
//
//        double flPower = 0, frPower=0, blPower=0, brPower=0;
//
//        if (direction.equalsIgnoreCase("LEFT")) {
//            flPower = -power;
//            frPower = power;
//            blPower = power;
//            brPower = -power;
//
//        } else if (direction.equalsIgnoreCase("RIGHT")) {
//            flPower = power;
//            frPower = -power;
//            blPower = -power;
//            brPower = power;
//
//        } else if (direction.equalsIgnoreCase("BACK")) {
//            flPower = -power;
//            frPower = -power;
//            blPower = -power;
//            brPower = -power;
//    } else if (direction.equalsIgnoreCase("FORWARD")) {
//            flPower = power;
//            frPower = power;
//            blPower = power;
//            brPower = power;
//        }
//        if(direction == ""){
//            // Set target position
//            frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + targetCounts );
//            frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + targetCounts );
//            backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + targetCounts );
//            backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + targetCounts );
//        }
//        else {
//            // Set target position
//            frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + (int) (targetCounts * flPower));
//            frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + (int) (targetCounts * frPower));
//            backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + (int) (targetCounts * blPower));
//            backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + (int) (targetCounts * brPower));
//
//        }
//        // Switch to RUN_TO_POSITION mode
//        setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Start moving
//        setPower(speed);
//
//        // Wait until target is reached
//        //opModeIsActive() &&
//        while (
//                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
//                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {
//
//            // Calculate current distance traveled
//            double avgPosition = (frontLeftDrive.getCurrentPosition() +
//                    frontRightDrive.getCurrentPosition() +
//                    backLeftDrive.getCurrentPosition() +
//                    backRightDrive.getCurrentPosition()) / 4.0;
//            double distanceTraveled = avgPosition / COUNTS_PER_INCH;
//
//            // Update current pose based on heading and distance
//            Pose2d currentPose = new Pose2d(
//                    startPose.x + distanceTraveled * Math.sin(robotPose.heading),
//                    startPose.y + distanceTraveled * Math.cos(robotPose.heading),
//                    robotPose.heading
//            );
//
//            telemetry.addData("HeadingDeg", "Driving:",getPoseHeadingDeg());
//
//            telemetry.addData("Status", "Driving Forward", getPoseHeadingDeg());
//            telemetry.addData("Target Distance", "%.2f inches", inches);
//            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
//            telemetry.addLine();
//            telemetry.addData("Current Pose", currentPose.toString());
//            telemetry.addData("X Position", "%.2f inches", currentPose.x);
//            telemetry.addData("Y Position", "%.2f inches", currentPose.y);
//            telemetry.addData("Heading", "%.1f°", currentPose.getHeadingDegrees());
//            telemetry.addLine();
//            telemetry.addData("Left Front", "%d / %d", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition());
//            telemetry.addData("Right Front", "%d / %d", frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition());
//            telemetry.update();
//        }
//
//        // Update final robot pose
//        robotPose.x += inches * Math.sin(robotPose.heading);
//        robotPose.y += inches * Math.cos(robotPose.heading);
//
//        // Stop motors
//        setPower(0);
//
//        // Switch back to RUN_USING_ENCODER
//        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    // ---------- 3. driveDistance with this ----------
    /**
     * Drives straight (or strafes) and keeps the robot square. Blocking.
     * @param direction "FORWARD", "BACK", "LEFT", "RIGHT" or "" (= FORWARD). null counts as "".
     * @param inches    distance. NEGATIVE reverses the direction ("LEFT", -26 = right 26).
     * @param speed     0-1 top power (ramps up from a minimum and slows near the end)
     */
    public void driveDistance(String direction, double inches, double speed) {
        String dir = (direction == null) ? "" : direction.trim();
        double fwd = 0, side = 0;                       // side + = RIGHT
        if (dir.isEmpty() || dir.equalsIgnoreCase("FORWARD")) fwd = 1;
        else if (dir.equalsIgnoreCase("BACK"))                fwd = -1;
        else if (dir.equalsIgnoreCase("RIGHT"))               side = 1;
        else if (dir.equalsIgnoreCase("LEFT"))                side = -1;
        else {
            telemetry.addData("driveDistance ERROR", "Unknown direction: " + direction);
            telemetry.update();
            return;
        }
        if (inches < 0) {                               // negative distance = the opposite way
            fwd = -fwd;
            side = -side;
            inches = -inches;
        }
        double maxPower = Math.min(1.0, Math.abs(speed));
        if (inches == 0 || maxPower == 0) {
            return;
        }

        boolean strafing = side != 0;
        double ticksPerInch = COUNTS_PER_INCH * (strafing ? STRAFE_TICKS_FACTOR : 1.0);
        double targetTicks = inches * ticksPerInch;
        double minPower = Math.min(strafing ? STRAFE_MIN_POWER : MIN_POWER, maxPower);
        double trim = side > 0 ? RIGHT_FORWARD_TRIM : (side < 0 ? LEFT_FORWARD_TRIM : 0.0);

        // ===== FIX A: absolute heading + pre-align (cures a CROOKED START) =====
        // PROBLEM: a strafe is only straight if the robot starts square. If the robot begins
        //   5 degrees off, a "straight" strafe goes 5 degrees off the line, and the error grows
        //   with distance (65 in at 5 degrees ends about 5.7 in off target).
        //   An earlier version saved the heading at the START of each move, which just copied
        //   the crooked angle and held it crooked.
        // FIX: hold the ABSOLUTE heading in holdHeading. 0 = the angle the robot had at INIT
        //   (imu.resetYaw() in init()), so "straight" means "square to the field", not
        //   "however the robot happened to be pointing when this move began".
        //   If the robot is more than PRE_ALIGN_DEG away from it, turnToHeading() squares the
        //   robot first, THEN the strafe starts.
        //   CAUTION: turnDegrees() does not update holdHeading in this file. If you rotate on
        //   purpose and then call driveDistance(), it turns the robot back to the old heading.
        //   Fix: add "holdHeading = yawDeg();" at the end of turnDegrees(), or call
        //   setHoldHeading(angle) after a deliberate turn.
        // NOTE: the robot must be placed square BEFORE pressing INIT (it must not move after).
        final double hold = holdHeading;
        if (Math.abs(AngleUnit.normalizeDegrees(hold - yawDeg())) > PRE_ALIGN_DEG) {
            turnToHeading(hold);
        }

        // Encoder starts are read AFTER the pre-align turn.
        final int flStart = frontLeftDrive.getCurrentPosition();
        final int frStart = frontRightDrive.getCurrentPosition();
        final int blStart = backLeftDrive.getCurrentPosition();
        final int brStart = backRightDrive.getCurrentPosition();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        final double timeout = 1.5 + 0.15 * inches / Math.max(maxPower, 0.2);
        double traveledIn = 0, headingError = 0, driftIn = 0;

        while (isActive.getAsBoolean() && timer.seconds() < timeout) {
            int dFL = frontLeftDrive.getCurrentPosition()  - flStart;
            int dFR = frontRightDrive.getCurrentPosition() - frStart;
            int dBL = backLeftDrive.getCurrentPosition()   - blStart;
            int dBR = backRightDrive.getCurrentPosition()  - brStart;

            double fwdTicks  = (dFL + dFR + dBL + dBR) / 4.0;    // + = moved ahead
            double sideTicks = (dFL - dFR - dBL + dBR) / 4.0;    // + = moved right
            double traveledTicks = fwd * fwdTicks + side * sideTicks;
            if (traveledTicks >= targetTicks) {
                break;
            }

            traveledIn = Math.max(0.0, traveledTicks / ticksPerInch);   // never negative
            double remainingIn = inches - traveledIn;                    // always > 0 here

            // Ramp up from minPower, ramp down to minPower, never below it or above maxPower.
            double span = maxPower - minPower;
            double up   = minPower + span * Math.min(1.0, traveledIn / RAMP_UP_IN);
            double down = minPower + span * Math.min(1.0, remainingIn / RAMP_DOWN_IN);
            double p = Math.max(minPower, Math.min(up, down));

            // ===== FIX B: IMU heading hold (cures ROTATION while moving) =====
            // PROBLEM: while strafing, uneven grip, weight and motor strength make one side of
            //   the robot push harder, so the nose slowly rotates. The wheels then push along
            //   the rotated direction, so the path bends into a diagonal. The old code had no
            //   feedback, so nothing noticed or corrected the rotation.
            // FIX: every loop (about every few ms) read the IMU yaw, compare it with the held
            //   heading, and add a small turn command to cancel the difference (a P controller):
            //       turn = -HEADING_KP * (hold - yaw)
            //   Sign: yaw + = turned LEFT. error > 0 means the robot has yawed RIGHT of the
            //   target, so the turn is negative = rotate LEFT, back to square.
            //   The turn is clipped to MAX_HEADING_CORRECTION so it can never fight the strafe.
            //   Tuning: raise HEADING_KP if the end heading error is large, lower it if the
            //   robot wobbles left/right.
            // LIMIT: yaw only tells which way the nose points, not where the robot is. A robot
            //   can face perfectly straight and still slide forward/back. That is FIX C.
            headingError = AngleUnit.normalizeDegrees(hold - yawDeg());
            double turn = Math.max(-MAX_HEADING_CORRECTION,
                    Math.min(MAX_HEADING_CORRECTION, -HEADING_KP * headingError));

            // ===== FIX C: cross-track creep correction (cures SIDEWAYS-TO-FORWARD SLIDE) =====
            // PROBLEM: in a strafe, the roller forces in the forward/back direction should
            //   cancel, but with mismatched grip, weight or motors they do not. The robot
            //   creeps forward or back while facing straight (heading error about 0), so
            //   FIX B cannot see it, yet the path is still diagonal.
            // FIX: use the wheel encoders to measure the motion we did NOT ask for.
            //   Mecanum kinematics split the four wheel readings into two parts:
            //       fwdTicks  = (FL + FR + BL + BR) / 4   -> motion ahead/back
            //       sideTicks = (FL - FR - BL + BR) / 4   -> motion left/right
            //   While strafing, fwdTicks should stay 0; any value is creep (driftIn, inches,
            //   + = ahead). While driving forward/back, sideTicks is the creep (+ = right).
            //   Then push against it:  cross = -CROSS_KP * driftIn, clipped to MAX_CROSS.
            //   The correction is added to the forward command when strafing (or the strafe
            //   command when driving).
            //   trim (LEFT_FORWARD_TRIM / RIGHT_FORWARD_TRIM) is a fixed bias that cancels the
            //   robot's usual lean up front, so the loop only fixes what is left.
            // LIMIT: encoders only see wheel rotation, not real floor slip, so this cuts the
            //   creep but cannot remove all of it. If the robot still leans with heading error
            //   about 0, tune the TRIM constants (or add odometry pods / a Pinpoint).
            driftIn = strafing ? fwdTicks / COUNTS_PER_INCH
                    : sideTicks / (COUNTS_PER_INCH * STRAFE_TICKS_FACTOR);
            double cross = Math.max(-MAX_CROSS, Math.min(MAX_CROSS, -CROSS_KP * driftIn));

            double f = fwd * p + (strafing ? trim * p + cross : 0.0);
            double s = side * p + (strafing ? 0.0 : cross);
            setWheelsLinear(f, s, turn);

            telemetry.addData("Move", "%s %.1f in at %.2f", dir.isEmpty() ? "FORWARD" : dir, inches, p);
            telemetry.addData("Traveled", "%.1f in", traveledIn);
            telemetry.addData("Heading hold", "%.1f° (error %.1f°, turn %.2f)", hold, headingError, turn);
            telemetry.addData("Creep", "%.1f in (correction %.2f)", driftIn, cross);
            telemetry.update();
        }

        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("driveDistance done", "%.1f of %.1f in, heading error %.1f°, creep %.1f in%s",
                traveledIn, inches, headingError, driftIn, timer.seconds() >= timeout ? "  TIMEOUT" : "");
        telemetry.update();

        if (fwd != 0) {
            robotPose.x += fwd * traveledIn * Math.sin(robotPose.heading);
            robotPose.y += fwd * traveledIn * Math.cos(robotPose.heading);
        }
    }


    private double yawDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);   // + = turned LEFT
    }

    // Plain linear mecanum mix (+ turnRight = clockwise), NOT the cubic curve that drive() applies.
    private void setWheelsLinear(double forward, double strafe, double turnRight) {
        double fl = forward + strafe + turnRight;
        double fr = forward - strafe - turnRight;
        double bl = forward - strafe + turnRight;
        double br = forward + strafe - turnRight;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    // NEW: paste it here
    private void setMecanum(double f, double s, double r) {
        double fl = f + s + r;
        double fr = f - s - r;
        double bl = f - s + r;
        double br = f + s - r;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    public void turnToHeading(){
        turnToHeading(holdHeading);
    }
    // Turn on the spot until the nose points at targetDeg (gives up after 1.5 s)
    private void turnToHeading(double targetDeg) {
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < 1.5) {
            double err = AngleUnit.normalizeDegrees(targetDeg - getPoseHeadingDeg());
            if (Math.abs(err) < 0.7) break;
            double turn = Range.clip(-0.03 * err, -0.3, 0.3);          // + = clockwise
            if (Math.abs(turn) < 0.1) turn = Math.copySign(0.1, turn);  // enough to actually move
            frontLeftDrive.setPower(turn);   backLeftDrive.setPower(turn);
            frontRightDrive.setPower(-turn); backRightDrive.setPower(-turn);
        }
        setPower(0);
    }

    public void turnDegrees(double degrees, double speed) {
        // Calculate distance each wheel needs to travel
        // Adjust ROBOT_WIDTH based on your robot's width in inches
        double ROBOT_WIDTH = 17.0;  // Distance between left and right wheels
        double arcLength = Math.PI * ROBOT_WIDTH * (degrees / 360.0);
        int targetCounts = (int)(arcLength * COUNTS_PER_INCH);

        // Store starting heading
        double startHeading = robotPose.heading;

        // Set target positions (left and right wheels move opposite directions)
        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + targetCounts);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - targetCounts);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + targetCounts);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - targetCounts);

        // Switch to RUN_TO_POSITION mode
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start turning
        setPower(speed);

        // Wait until target is reached
        //opModeIsActive()
        while (
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {

            // Calculate current turn progress
            double avgLeftPosition = (frontLeftDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition()) / 2.0;
            double distanceTraveled = avgLeftPosition / COUNTS_PER_INCH;
            double currentTurnDegrees = (distanceTraveled / arcLength) * degrees;
            double currentTurnRadians = Math.toRadians(currentTurnDegrees);

            // Create current pose for telemetry
            Pose2d currentPose = new Pose2d(
                    robotPose.x,
                    robotPose.y,
                    startHeading + currentTurnRadians
            );

            String direction = degrees > 0 ? "Left" : "Right";
            telemetry.addData("Status", "Turning %s", direction);
            telemetry.addData("Target Angle", "%.1f°", degrees);
            telemetry.addData("Current Turn", "%.1f°", currentTurnDegrees);
            telemetry.addLine();
            telemetry.addData("Current Pose", currentPose.toString());
            telemetry.addData("X Position", "%.2f inches", currentPose.x);
            telemetry.addData("Y Position", "%.2f inches", currentPose.y);
            telemetry.addData("Heading", "%.1f°", currentPose.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Left Motors", "%d / %d", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition());
            telemetry.addData("Right Motors", "%d / %d", frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition());
            telemetry.update();
        }

        // Update pose heading
        robotPose.heading += Math.toRadians(degrees);
        // Normalize heading to -PI to PI
        while (robotPose.heading > Math.PI) robotPose.heading -= 2 * Math.PI;
        while (robotPose.heading < -Math.PI) robotPose.heading += 2 * Math.PI;

        // Stop motors
        setPower(0);

        // Switch back to RUN_USING_ENCODER
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Helper method for basic timed strafing with power compensation.
     * @param power The absolute base power level (0.0 to 1.0)
     * @param direction "LEFT" or "RIGHT"
     * @param durationMs The time to strafe in milliseconds
     */
    public void strafe(double power, String direction, double durationMs) {

        double flPower, frPower, blPower, brPower;

        if (direction.equalsIgnoreCase("LEFT")) {
            flPower = -power;
            frPower = power;
            blPower = power;
            brPower = -power;

        } else if (direction.equalsIgnoreCase("RIGHT")) {
            flPower = power;
            frPower = -power;
            blPower = -power;
            brPower = power;

        } else {
            //stopDrive();
            return;
        }



        // --- APPLY COMPENSATION FACTORS HERE ---
        // This is the key change to adjust for asymmetries.
        frontLeftDrive.setPower(flPower * FL_COMPENSATION);
        frontRightDrive.setPower(frPower * FR_COMPENSATION);
        backLeftDrive.setPower(blPower * BL_COMPENSATION);
        backRightDrive.setPower(brPower * BR_COMPENSATION);

        telemetry.addData("Strafe", "%s at %.2f power", direction, power);
        telemetry.addData("FL Power", "%.2f", flPower * FL_COMPENSATION);
        // ... (Add telemetry for all four motors)
        telemetry.update();

        // Wait for the duration
        //sleep(durationMs);

        // Stop motors after duration
       // stopDrive();
    }

    public boolean  mecanumFSRDrive(ElapsedTime driveTimer, double forward, double strafe, double rotate, double holdSeconds) {

        // --- Configuration Constants ---
        // Max power robot can reach (e.g., 0.8 for 80% of full speed)
        final double MAX_SPEED_FACTOR = 0.5;
        // Power Smoothing Exponent: 1.0 is linear, 3.0 is cubic (more sensitive to small inputs)
        final double POWER_SMOOTHING_EXPONENT = 3.0;

        // --- 1. Apply Smoothing (Non-Linear Power Curve) ---
        // Uses the formula: sign(x) * |x|^P, which gives finer control at low speeds.
        double f = Math.copySign(Math.pow(Math.abs(forward), POWER_SMOOTHING_EXPONENT), forward);
        double s = Math.copySign(Math.pow(Math.abs(strafe), POWER_SMOOTHING_EXPONENT), strafe);
        double r = Math.copySign(Math.pow(Math.abs(rotate), POWER_SMOOTHING_EXPONENT), rotate);

        // --- 2. Calculate Raw Motor Powers (Mecanum Kinematics) ---
        /* The signs for 's' are crucial. The standard set is (+s, -s, -s, +s).
         * If strafing is reversed, change all 's' signs.
         * e.g., for LeftFront use (f - s + r) instead of (f + s + r)
         */

        double leftFrontRawPower = f + s + r;
        double rightFrontRawPower = f - s - r;
        double leftBackRawPower = f - s + r;
        double rightBackRawPower = f + s - r;


//        // Option B: Use this if strafing is reversed with the standard signs.
//        double leftFrontRawPower = f - s + r; // <-- s sign inverted
//        double rightFrontRawPower = f + s - r; // <-- s sign inverted
//        double leftBackRawPower = f + s + r; // <-- s sign inverted
//        double rightBackRawPower = f - s - r; // <-- s sign inverted

        // --- 3. Normalization (Scaling) ---
        /* Denominator is the largest absolute power required across all motors, or 1.
         * This ensures no power value exceeds 1.0 *before* the MAX_SPEED_FACTOR is applied.
         */
        double maxMagnitude = Math.max(
                Math.abs(leftFrontRawPower),
                Math.max(
                        Math.abs(rightFrontRawPower),
                        Math.max(Math.abs(leftBackRawPower), Math.abs(rightBackRawPower))
                )
        );
        // Use Math.max(maxMagnitude, 1) to prevent division by a number less than 1,
        // which would unnecessarily scale up small powers (though power smoothing makes this less likely)
        double denominator = Math.max(maxMagnitude, 1.0);

        // --- 4. Final Power Calculation and Limiting ---

        // Calculate final power and scale by the MAX_SPEED_FACTOR
        double leftFrontPower = (leftFrontRawPower / denominator) * MAX_SPEED_FACTOR;
        double rightFrontPower = (rightFrontRawPower / denominator) * MAX_SPEED_FACTOR;
        double leftBackPower = (leftBackRawPower / denominator) * MAX_SPEED_FACTOR;
        double rightBackPower = (rightBackRawPower / denominator) * MAX_SPEED_FACTOR;

        // --- 5. Set Motor Powers ---
        frontLeftDrive.setPower(leftFrontPower);
        frontRightDrive.setPower(rightFrontPower);
        backLeftDrive.setPower(leftBackPower);
        backRightDrive.setPower(rightBackPower);
        return (driveTimer.seconds() > holdSeconds);


    }
    // Helper methods
    private void setMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    private void setPower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    public void stop(){
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(BRAKE);
        }
    }
}
