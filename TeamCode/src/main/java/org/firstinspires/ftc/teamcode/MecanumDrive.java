package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class MecanumDrive {

    private double FL_COMPENSATION=0.0;
    private double      FR_COMPENSATION=0.0;
    private double  BL_COMPENSATION=0.0;
    private double        BR_COMPENSATION=0.0;
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

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, flDrive);
        frontRightDrive = hardwareMap.get(DcMotor.class, frDrive);
        backLeftDrive = hardwareMap.get(DcMotor.class, blDrive);
        backRightDrive = hardwareMap.get(DcMotor.class, brDrive);
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

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
        double maxSpeed = 1.0;  // make this slower for outreaches

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
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
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

    public void driveDistance(String direction, double inches, double speed) {
        int targetCounts = (int)(inches * COUNTS_PER_INCH);
double power = 0.6;
        // Store starting pose
        Pose2d startPose = robotPose.copy();

        double flPower = 0, frPower=0, blPower=0, brPower=0;

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

        }
if(direction == ""){
    // Set target position
    frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + targetCounts );
    frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + targetCounts );
    backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + targetCounts );
    backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + targetCounts );
}
else {
    // Set target position
    frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + (int) (targetCounts * flPower));
    frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + (int) (targetCounts * frPower));
    backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + (int) (targetCounts * blPower));
    backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + (int) (targetCounts * brPower));

}
        // Switch to RUN_TO_POSITION mode
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        setPower(speed);

        // Wait until target is reached
        //opModeIsActive() &&
        while (
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                        backLeftDrive.isBusy() || backRightDrive.isBusy())) {

            // Calculate current distance traveled
            double avgPosition = (frontLeftDrive.getCurrentPosition() +
                    frontRightDrive.getCurrentPosition() +
                    backLeftDrive.getCurrentPosition() +
                    backRightDrive.getCurrentPosition()) / 4.0;
            double distanceTraveled = avgPosition / COUNTS_PER_INCH;

            // Update current pose based on heading and distance
            Pose2d currentPose = new Pose2d(
                    startPose.x + distanceTraveled * Math.sin(robotPose.heading),
                    startPose.y + distanceTraveled * Math.cos(robotPose.heading),
                    robotPose.heading
            );

            telemetry.addData("Status", "Driving Forward");
            telemetry.addData("Target Distance", "%.2f inches", inches);
            telemetry.addData("Distance Traveled", "%.2f inches", distanceTraveled);
            telemetry.addLine();
            telemetry.addData("Current Pose", currentPose.toString());
            telemetry.addData("X Position", "%.2f inches", currentPose.x);
            telemetry.addData("Y Position", "%.2f inches", currentPose.y);
            telemetry.addData("Heading", "%.1f°", currentPose.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Left Front", "%d / %d", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition());
            telemetry.addData("Right Front", "%d / %d", frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition());
            telemetry.update();
        }

        // Update final robot pose
        robotPose.x += inches * Math.sin(robotPose.heading);
        robotPose.y += inches * Math.cos(robotPose.heading);

        // Stop motors
        setPower(0);

        // Switch back to RUN_USING_ENCODER
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnDegrees(double degrees, double speed) {
        // Calculate distance each wheel needs to travel
        // Adjust ROBOT_WIDTH based on your robot's width in inches
        double ROBOT_WIDTH = 16.0;  // Distance between left and right wheels
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

}
