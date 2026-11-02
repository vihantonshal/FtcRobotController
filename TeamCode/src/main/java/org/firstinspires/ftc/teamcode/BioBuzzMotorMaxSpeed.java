package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * A tuning tool: measures how fast one motor can spin, and works out starting PIDF numbers
 * for it. Use it for the drive motors, the intake and the launcher.
 *
 * PUT THE ROBOT ON A BLOCK FIRST so the wheels are off the ground.
 *
 * Controls (gamepad 1):
 *   D-pad up / down : pick a motor
 *   A (hold)        : run the picked motor at full power
 *   B               : clear the top speed
 *
 * Hold A for 3-4 seconds so the motor reaches full speed. Then copy the numbers it shows into
 * that part's subsystem: PIDF and MAX_TICKS_PER_SEC in subsystems/MecanumDriveSubsystem or
 * subsystems/IntakeSubsystem, PIDF in subsystems/LauncherSubsystem. The numbers follow the FTC motor
 * PIDF tuning guide:
 *   F = 32767 / top speed    P = 0.1 * F    I = 0.1 * P    D = 0
 * Use a well-charged battery, because a tired battery gives a lower top speed.
 */
@TeleOp(name = "BioBuzz Motor Max Speed", group = "StarterBot")
public class BioBuzzMotorMaxSpeed extends OpMode {

    // Names from the Driver Hub configuration, the same ones the StarterBot programs use.
    // The intake and launcher are left out when their switch in RobotSwitches is off.
    private String[] motorNames;

    private DcMotorEx[] motors;
    private int selected = 0;
    private double topSpeed = 0;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<String> names = new ArrayList<>();
        if (RobotSwitches.USE_DRIVE) {
            names.addAll(Arrays.asList("front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor"));
        }
        if (RobotSwitches.USE_INTAKE) {
            names.add("intake");
        }
        if (RobotSwitches.USE_LAUNCHER) {
            names.add("launcher");
        }
        motorNames = names.toArray(new String[0]);
        motors = new DcMotorEx[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            // Plain power, no PIDF: we want to see what the motor does on its own.
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addLine("Put the robot on a block. Pick a motor with the D-pad, hold A to run it.");
    }

    @Override
    public void loop() {
        if (motors.length == 0) {
            telemetry.addLine("No motors: USE_DRIVE, USE_INTAKE and USE_LAUNCHER are all off in RobotSwitches.");
            telemetry.update();
            return;
        }
        // Pick a motor. We only react when the button is first pressed, not every loop it is held.
        if (gamepad1.dpad_up && !lastUp) {
            selectMotor((selected + motors.length - 1) % motors.length);
        }
        if (gamepad1.dpad_down && !lastDown) {
            selectMotor((selected + 1) % motors.length);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        if (gamepad1.b) {
            topSpeed = 0;
        }

        DcMotorEx motor = motors[selected];
        motor.setPower(gamepad1.a ? 1 : 0);

        double speed = Math.abs(motor.getVelocity());
        topSpeed = Math.max(topSpeed, speed);

        telemetry.addData("Motor", "%s (D-pad to change)", motorNames[selected]);
        telemetry.addData("speed now", speed);
        telemetry.addData("top speed", topSpeed);
        if (topSpeed > 0) {
            double f = 32767 / topSpeed;
            double p = 0.1 * f;
            double i = 0.1 * p;
            telemetry.addData("Suggested PIDF", "P %.3f, I %.4f, D 0, F %.2f", p, i, f);
            telemetry.addData("Suggested max ticks/s", "%.0f (90%% of top speed)", 0.9 * topSpeed);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    // Stop the old motor and start measuring a new one from zero.
    private void selectMotor(int index) {
        motors[selected].setPower(0);
        selected = index;
        topSpeed = 0;
    }
}
