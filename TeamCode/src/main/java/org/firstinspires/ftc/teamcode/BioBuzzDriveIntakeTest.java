package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/*
 * DRIVE + INTAKE TEST: checks that every wheel and the intake turn the right way, and that odometry
 * counts. PUT THE ROBOT ON A BLOCK first (wheels off the floor), then press INIT and START.
 *
 * Gamepad 1, hold:
 *   D-pad up / down       all wheels forward / backward                  (at TEST_POWER)
 *   D-pad right / left    strafe right / left
 *   right / left bumper   turn right / left
 *   nothing held          the sticks drive, like the TeleOp (left = drive/strafe, right X = turn)
 *   right / left trigger  intake in / out
 *
 * For each D-pad / bumper test, every wheel shows its speed and OK or WRONG WAY: a wheel that turns
 * the wrong way has its setDirection() wrong in MecanumDriveSubsystem (or its encoder counts backward).
 * The intake shows OK when "in" gives a + speed. The odometry pose should grow in the direction tested
 * (forward = x up, strafe left = y up, turn left = heading up).
 *
 * Needs RobotSwitches.USE_DRIVE and USE_INTAKE on; a part that is off shows OFF and does nothing.
 *
 * DASHBOARD
 *   Graph:          each wheel's "... velocity" against "... target", "intake velocity" against
 *                   "intake target" (the targets show while RobotSwitches.USE_DASHBOARD is on).
 *   Configuration:  BioBuzzDriveIntakeTest  TEST_POWER, MIN_SPEED (below)
 *                   MecanumDriveSubsystem   PIDF, MAX_TICKS_PER_SEC, TELEOP_SPEED_SCALE, SLOW_MODE_SCALE,
 *                                           MAX_ACCEL_PER_SEC, DEADBAND, STRAFE_MULTIPLIER, TRACK_WIDTH_MM
 *                   IntakeSubsystem         PIDF, MAX_TICKS_PER_SEC
 *   A changed PIDF reaches the motors within one loop. Changes are lost when the robot restarts: copy
 *   the keepers into the .java files.
 */
@Config
@TeleOp(name = "Drive + Intake Test", group = "StarterBot")
@Disabled
public class BioBuzzDriveIntakeTest extends OpMode {

    // Power for the D-pad / bumper tests (0-1 of MAX_TICKS_PER_SEC). Low, so the robot can't run away.
    public static double TEST_POWER = 0.3;
    // A wheel slower than this (ticks/s) counts as "not turning".
    public static double MIN_SPEED = 50;

    private static final String[] WHEELS = {"front left", "front right", "back left", "back right"};

    private MecanumDriveSubsystem drive = null;
    private IntakeSubsystem intake = null;

    @Override
    public void init() {
        // Test OpModes always send to FTC Dashboard: they never run in a match.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        telemetry.addData("Drive", drive.isEnabled() ? "on" : "OFF: set RobotSwitches.USE_DRIVE = true");
        telemetry.addData("IMU", drive.hasImu() ? "found" : "missing (heading from the wheels)");
        telemetry.addData("Intake", intake.isEnabled() ? "on" : "OFF: set RobotSwitches.USE_INTAKE = true");
        telemetry.addLine("PUT THE ROBOT ON A BLOCK, then START. D-pad / bumpers test the wheels, triggers the intake.");
    }

    @Override
    public void start() {
        drive.startTeleop();
    }

    @Override
    public void loop() {
        drive.update();   // odometry

        // ---------------- DRIVE ----------------
        // The test pattern held now: forward, strafe (+ = right), rotate (+ = right), as in drive().
        double f = 0, s = 0, r = 0;
        String test;
        if (gamepad1.dpad_up) {
            f = TEST_POWER; test = "forward";
        } else if (gamepad1.dpad_down) {
            f = -TEST_POWER; test = "backward";
        } else if (gamepad1.dpad_right) {
            s = TEST_POWER; test = "strafe right";
        } else if (gamepad1.dpad_left) {
            s = -TEST_POWER; test = "strafe left";
        } else if (gamepad1.right_bumper) {
            r = TEST_POWER; test = "turn right";
        } else if (gamepad1.left_bumper) {
            r = -TEST_POWER; test = "turn left";
        } else {
            test = null;
        }

        if (!drive.isEnabled()) {
            telemetry.addData("DRIVE", "OFF (RobotSwitches.USE_DRIVE = false)");
        } else if (test == null) {
            drive.driveWithSticks(gamepad1);
            telemetry.addData("DRIVE", "sticks (hold a D-pad direction or a bumper to test)");
        } else {
            drive.drive(f, s, r);
            telemetry.addData("DRIVE", "testing %s at %.0f%%", test, TEST_POWER * 100);
        }

        // Each wheel's speed, and with a test held, whether it turns the way the mecanum formula says.
        double[] expected = {f + s + r, f - s - r, f - s + r, f + s - r};
        double[] speed = drive.getWheelVelocities();
        for (int i = 0; i < 4; i++) {
            telemetry.addData("  " + WHEELS[i], "%6.0f ticks/s %s", speed[i],
                    test == null ? "" : wheelCheck(expected[i], speed[i]));
        }
        telemetry.addData("  odometry", "x %.1f in, y %.1f in, heading %.1f°",
                drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg());

        // ---------------- INTAKE ----------------
        double power = gamepad1.right_trigger - gamepad1.left_trigger;   // + = in
        intake.runTeleop(power, false);
        if (!intake.isEnabled()) {
            telemetry.addData("INTAKE", "OFF (RobotSwitches.USE_INTAKE = false)");
        } else {
            telemetry.addData("INTAKE", "%s, %.0f of %.0f ticks/s %s",
                    power > 0 ? "in" : power < 0 ? "out" : "stopped", intake.getVelocity(),
                    intake.getTargetVelocity(), power == 0 ? "" : wheelCheck(power, intake.getVelocity()));
        }

        // The velocity/target lines for the FTC Dashboard graphs.
        drive.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        telemetry.update();
    }

    // OK when the motor turns the way it was asked (same sign), "not turning" when it barely moves.
    private String wheelCheck(double asked, double speed) {
        if (Math.abs(asked) < 1e-6) {
            return "(should stay still)";
        }
        if (Math.abs(speed) < MIN_SPEED) {
            return "NOT TURNING";
        }
        return Math.signum(speed) == Math.signum(asked) ? "OK" : "WRONG WAY";
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (intake != null) {
            intake.stop();
        }
    }
}
