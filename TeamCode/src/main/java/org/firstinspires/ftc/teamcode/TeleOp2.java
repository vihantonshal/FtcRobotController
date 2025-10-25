package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@TeleOp(name = "TeleOp2")
public class TeleOp2 extends LinearOpMode {



    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive MCDrive  = new MecanumDrive(hardwareMap, telemetry);

        if (opModeIsActive()) {
            MCDrive.init();

            // Pre-run
            while (opModeIsActive()) {
                telemetry.addLine("Press A to reset Yaw");
                telemetry.addLine("Hold left bumper to drive in robot relative");
                telemetry.addLine("The left joystick sets the robot direction");
                telemetry.addLine("Moving the right joystick left and right turns the robot");

                // OpMode loop
                // If you press the A button, then you reset the Yaw to be zero from the way
                // the robot is currently pointing
                if (gamepad1.a) {
                    MCDrive.imu.resetYaw();
                }
                // If you press the left bumper, you get a drive from the point of view of the robot
                // (much like driving an RC vehicle)
                if (gamepad1.left_bumper) {
                    MCDrive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                } else {
                    MCDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


                }
            }

        }
    }
}
