package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/*
 * LAUNCHER TEST: spin the flywheel, feed balls, and TUNE it live in FTC Dashboard
 * (http://192.168.43.1:8080/dash). Uses the same LauncherSubsystem as the TeleOp and the Auto.
 *
 * Gamepad 1:
 *   right bumper      launcher on / off
 *   A (hold)          feed (the roller motor "roller" waits until the flywheel is at speed, then feeds in pulses)
 *   D-pad right/left  trim the speed + / - TRIM_STEP
 *   B                 clear the measurements
 *
 * DASHBOARD
 *   Graph:          "launcher velocity" and "launcher target" (and "launcher error") on one graph.
 *   Configuration:  LauncherSubsystem   PIDF, TARGET_VELOCITY, SPEED_SCALE, FEED_MARGIN, FEED_MAX_OVER,
 *                                       FEED_ON/OFF_SECONDS, USE_DISTANCE_SPEED, NEAR/FAR ...
 *                   BioBuzzLauncherTest TEST_RANGE_IN, PULSE_FEED (below)
 *   A changed number reaches the motor within one loop. Dashboard changes are lost when the robot
 *   restarts: copy the numbers you keep into LauncherSubsystem.java.
 *
 * WHAT TO LOOK AT (the "Tune" line gives a hint):
 *   spin-up   time from "on" to first at speed. Slow -> raise PIDF f (most of the speed comes from f).
 *   overshoot how far above the target it went. More than FEED_MAX_OVER -> lower p.
 *   dip       the lowest speed while feeding: each ball slows the flywheel.
 *   recovery  time from a dip back to speed. Slow -> raise p a little (or FEED_OFF_SECONDS).
 *
 * Needs RobotSwitches.USE_LAUNCHER on. KEEP HANDS AWAY from the flywheel.
 */
@Config
@TeleOp(name = "Launcher Test", group = "StarterBot")
public class BioBuzzLauncherTest extends OpMode {

    // > 0: pretend the camera sees a CELL this far away (inches), to test the distance speed
    // (needs LauncherSubsystem.USE_DISTANCE_SPEED = true). 0 = no CELL.
    public static double TEST_RANGE_IN = 0;
    // Feed in pulses like the TeleOp and the Auto (false = feed the whole time A is held).
    public static boolean PULSE_FEED = true;

    private LauncherSubsystem launcher = null;

    // Measurements, cleared with B and every time the launcher is turned on.
    private final ElapsedTime onTimer = new ElapsedTime();
    private final ElapsedTime dipTimer = new ElapsedTime();
    private double spinUpSeconds = Double.NaN;
    private double overshoot = 0;
    private double dipLowest = Double.NaN;
    private double recoverySeconds = Double.NaN;
    private boolean inDip = false;

    @Override
    public void init() {
        // Test OpModes always send to FTC Dashboard: they never run in a match.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        launcher = new LauncherSubsystem(hardwareMap);
        telemetry.addData("Launcher", launcher.isEnabled() ? "on" : "OFF: set RobotSwitches.USE_LAUNCHER = true");
        telemetry.addData("Feeder", launcher.getFeederText());
        telemetry.addData("Roller", launcher.getRollerText());
        telemetry.addLine("Right bumper = on/off, hold A = feed. Tune in FTC Dashboard: LauncherSubsystem.");
    }

    @Override
    public void loop() {
        launcher.setPulseFeed(PULSE_FEED);
        if (TEST_RANGE_IN > 0) {
            launcher.setCellRange(TEST_RANGE_IN);
        }

        // ---------------- Buttons ----------------
        if (gamepad1.rightBumperWasPressed()) {
            launcher.toggleOn();
            clearMeasurements();
        }
        if (gamepad1.dpadRightWasPressed()) {
            launcher.adjustTrim(1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcher.adjustTrim(-1);
        }
        if (gamepad1.bWasPressed()) {
            clearMeasurements();
        }
        launcher.run(gamepad1.a);
        measure();

        // ---------------- Driver Hub ----------------
        double target = launcher.getShootTarget();
        double velocity = launcher.getVelocity();
        telemetry.addData("Launcher", launcher.isEnabled() ? launcher.getStateText() : "OFF (RobotSwitches.USE_LAUNCHER = false)");
        telemetry.addData("Speed", "%.0f of %.0f ticks/s (%.0f RPM), %s, feed window %.0f-%.0f",
                velocity, target, launcher.getRpm(), launcher.getDirection(),
                target - LauncherSubsystem.FEED_MARGIN, target + LauncherSubsystem.FEED_MAX_OVER);
        telemetry.addData("Target from", "%s, trim %+.0f",
                TEST_RANGE_IN > 0 && LauncherSubsystem.USE_DISTANCE_SPEED
                        ? String.format("distance %.0f in", TEST_RANGE_IN)
                        : String.format("TARGET_VELOCITY %.0f x SPEED_SCALE %.2f", LauncherSubsystem.TARGET_VELOCITY, LauncherSubsystem.SPEED_SCALE),
                launcher.getTrim());
        telemetry.addData("Feeder", launcher.getFeederText());
        telemetry.addData("Roller", launcher.getRollerText());
        telemetry.addData("Spin-up",Double.isNaN(spinUpSeconds) ? "-" : String.format("%.2f s", spinUpSeconds));
        telemetry.addData("Overshoot", "%+.0f ticks/s over the target", overshoot);
        telemetry.addData("Dip while feeding", Double.isNaN(dipLowest) ? "-" : String.format("lowest %.0f (%.0f below)", dipLowest, target - dipLowest));
        telemetry.addData("Recovery", Double.isNaN(recoverySeconds) ? "-" : String.format("%.2f s back to speed", recoverySeconds));
        telemetry.addData("Tune", hint());
        telemetry.addData("PIDF now", "p %.2f  i %.2f  d %.2f  f %.2f",
                LauncherSubsystem.PIDF.p, LauncherSubsystem.PIDF.i, LauncherSubsystem.PIDF.d, LauncherSubsystem.PIDF.f);

        // ---------------- Dashboard graph lines ----------------
        launcher.addTelemetry(telemetry);   // "launcher velocity", "launcher target"
        telemetry.addData("launcher error", velocity - target);
        telemetry.update();
    }

    // Spin-up time, overshoot, the lowest speed while feeding, and how long each dip takes to recover.
    private void measure() {
        if (!launcher.isOn()) {
            return;
        }
        double target = launcher.getShootTarget();
        double velocity = launcher.getVelocity();
        boolean atSpeed = launcher.isAtSpeed();
        if (Double.isNaN(spinUpSeconds)) {
            if (atSpeed) {
                spinUpSeconds = onTimer.seconds();
            }
            return;   // overshoot and dips count only after the first time at speed
        }
        overshoot = Math.max(overshoot, velocity - target);
        if (launcher.isFeeding() || inDip) {
            dipLowest = Double.isNaN(dipLowest) ? velocity : Math.min(dipLowest, velocity);
        }
        if (!atSpeed && velocity < target && !inDip) {
            inDip = true;         // a ball slowed it down
            dipTimer.reset();
        } else if (atSpeed && inDip) {
            inDip = false;
            recoverySeconds = dipTimer.seconds();
        }
    }

    // One hint for the kids, from the measurements.
    private String hint() {
        if (!launcher.isEnabled()) {
            return "turn on RobotSwitches.USE_LAUNCHER";
        }
        if (!launcher.isOn()) {
            return "right bumper to spin up";
        }
        if ("REVERSE".equals(launcher.getDirection())) {
            return "flywheel spins BACKWARD: check the motor direction or wiring";
        }
        if (Double.isNaN(spinUpSeconds)) {
            return onTimer.seconds() > 3 ? "not at speed after 3 s: raise PIDF f" : "spinning up...";
        }
        if (overshoot > LauncherSubsystem.FEED_MAX_OVER) {
            return "overshoots the feed window: lower PIDF p";
        }
        if (!Double.isNaN(recoverySeconds) && recoverySeconds > LauncherSubsystem.RECOVER_MAX_SECONDS) {
            return "slow to recover after a ball: raise PIDF p a little";
        }
        return "looks good: hold A to feed and watch the dip and recovery";
    }

    private void clearMeasurements() {
        onTimer.reset();
        spinUpSeconds = Double.NaN;
        overshoot = 0;
        dipLowest = Double.NaN;
        recoverySeconds = Double.NaN;
        inDip = false;
    }

    @Override
    public void stop() {
        if (launcher != null) {
            launcher.stop();
        }
    }
}
