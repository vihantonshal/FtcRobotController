package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonomous Distance Move ", group = "Autonomous")
public class AutonomousDistanceMove extends LinearOpMode {

    // Hardware
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Configuration
    private static final double COUNTS_PER_MOTOR_REV = 537.7;  // eg: GoBILDA 312 RPM
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Alliance configuration - CHANGE THIS
    private enum Alliance {
        BLUE,
        RED
    }
    private static final Alliance CURRENT_ALLIANCE = Alliance.BLUE;

    // Movement parameters
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final double DISTANCE_INCHES = 48.0;  // Distance to move forward
    private static final double TURN_ANGLE = 90.0;       // Degrees to turn (positive = left/ccw)

    // Pose2d for localization
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

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        telemetry.addData("Alliance", CURRENT_ALLIANCE);
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Robot Pose", robotPose.toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward the specified distance
            driveDistance(DISTANCE_INCHES, DRIVE_SPEED);
            sleep(500);

            // Turn based on alliance color
            if (CURRENT_ALLIANCE == Alliance.BLUE) {
                // Blue alliance turns left (positive angle)
                turnDegrees(TURN_ANGLE, TURN_SPEED);
            } else {
                // Red alliance turns right (negative angle)
                turnDegrees(-TURN_ANGLE, TURN_SPEED);
            }

            telemetry.addData("Status", "Complete");
            telemetry.addData("Final Pose", robotPose.toString());
            telemetry.addLine();
            telemetry.addData("X Position", "%.2f inches", robotPose.x);
            telemetry.addData("Y Position", "%.2f inches", robotPose.y);
            telemetry.addData("Heading", "%.1f°", robotPose.getHeadingDegrees());
            telemetry.update();
        }
    }

    private void initHardware() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "front_left_motor");
        rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        leftBack = hardwareMap.get(DcMotor.class, "back_left_motor");
        rightBack = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set motor directions (adjust based on your robot)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveDistance(double inches, double speed) {
        int targetCounts = (int)(inches * COUNTS_PER_INCH);

        // Store starting pose
        Pose2d startPose = robotPose.copy();

        // Set target position
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + targetCounts);

        // Switch to RUN_TO_POSITION mode
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        setPower(speed);

        // Wait until target is reached
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {

            // Calculate current distance traveled
            double avgPosition = (leftFront.getCurrentPosition() +
                    rightFront.getCurrentPosition() +
                    leftBack.getCurrentPosition() +
                    rightBack.getCurrentPosition()) / 4.0;
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
            telemetry.addData("Left Front", "%d / %d", leftFront.getCurrentPosition(), leftFront.getTargetPosition());
            telemetry.addData("Right Front", "%d / %d", rightFront.getCurrentPosition(), rightFront.getTargetPosition());
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

    private void turnDegrees(double degrees, double speed) {
        // Calculate distance each wheel needs to travel
        // Adjust ROBOT_WIDTH based on your robot's width in inches
        double ROBOT_WIDTH = 16.0;  // Distance between left and right wheels
        double arcLength = Math.PI * ROBOT_WIDTH * (degrees / 360.0);
        int targetCounts = (int)(arcLength * COUNTS_PER_INCH);

        // Store starting heading
        double startHeading = robotPose.heading;

        // Set target positions (left and right wheels move opposite directions)
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + targetCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - targetCounts);

        // Switch to RUN_TO_POSITION mode
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start turning
        setPower(speed);

        // Wait until target is reached
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {

            // Calculate current turn progress
            double avgLeftPosition = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2.0;
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
            telemetry.addData("Left Motors", "%d / %d", leftFront.getCurrentPosition(), leftFront.getTargetPosition());
            telemetry.addData("Right Motors", "%d / %d", rightFront.getCurrentPosition(), rightFront.getTargetPosition());
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

    // Helper methods
    private void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
}