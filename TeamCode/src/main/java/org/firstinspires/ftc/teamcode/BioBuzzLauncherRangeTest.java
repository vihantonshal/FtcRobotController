package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/*
1. Get the file onto your computer
adb connect 192.168.43.1:5555
adb pull /sdcard/FIRST/BioBuzz/launcher_range_test.csv

 * LAUNCHER RANGE TEST: try every combination of distance (from the camera) and flywheel speed,
 * and write every shot, HIT or MISS, to /sdcard/FIRST/BioBuzz/launcher_range_test.csv.
 * Its own file: test shots never become TeleOp launch poses (launch_log.csv is not touched).
 *
 * Only the camera and the launcher run: drive and intake are off unless USE_DRIVE / USE_INTAKE
 * (below) are turned on in FTC Dashboard before INIT. With the drive off, place the robot by hand.
 *
 * Gamepad 1 (one person):
 *   left stick / right stick X   drive and turn (slow mode on at start, X toggles)    USE_DRIVE only
 *   left bumper (hold)           turn toward the up CELL                              USE_DRIVE only
 *   right trigger / left trigger intake in / out                                      USE_INTAKE only
 *   right bumper                 launcher on / off
 *   D-pad right / left           speed + / - SPEED_STEP ticks/s
 *   D-pad up / down              speed + / - 4 x SPEED_STEP
 *   A (hold)                     feed: once the flywheel is at speed the roller runs without a
 *                                pause until A is released (same as the TeleOp)
 *   Y                            the last shot scored: HIT
 *   B                            the last shot missed: MISS
 *   One hold of A is one shot in the log, however many balls it launched. A shot not marked before
 *   the next one is written as UNMARKED. For one ball per row, load one ball or release A quickly.
 *
 * HOW TO TEST
 *   1. Move the robot until "Camera" shows the distance you want (e.g. 51, 55, 59, 63, 67 in) and
 *      aim it so "launcher" reads near 0°.
 *   2. Set a speed with the D-pad, wait for "READY", hold A, then Y or B.  3-5 balls per combination.
 *   3. Change the speed, repeat; then the next distance. "Results" shows hits/shots per combination.
 *   4. adb pull /sdcard/FIRST/BioBuzz/launcher_range_test.csv  and open it in a spreadsheet.
 *      Delete the file on the hub to start a fresh test.
 *
 * CSV columns: time_ms, shot, result, hive_cluster, range_in, bearing_deg, yaw_deg,
 *   launcher_bearing_deg, target_tps, feed_tps (measured when the feed started), feed_rpm,
 *   lowest_tps (the dip the ball caused), in_zone, camera_pitch_deg.  Range is the SDK's
 *   ftcPose.range, the same number LaunchAssist and the zone use. It is measured in the camera's own
 *   frame, so it changes if the mount is re-tilted: camera_pitch_deg (AprilTagVision.CAMERA_PITCH_DEG,
 *   the setting, not a measurement) records which tilt the data belongs to. Keep it equal to the real
 *   tilt (Camera Test's tilt finder) and don't re-tilt the camera between tests.
 *
 * The speed here is TARGET_VELOCITY x SPEED_SCALE + this test's offset (distance speed is turned
 * off while the test runs). Needs RobotSwitches.USE_LAUNCHER and USE_CAMERA on.
 */
@Config
@TeleOp(name = "Launcher Range Test", group = "StarterBot")
public class BioBuzzLauncherRangeTest extends OpMode {

    public static final String LOG_PATH = "BioBuzz/launcher_range_test.csv";
    private static final String HEADER = "time_ms,shot,result,hive_cluster,range_in,bearing_deg,yaw_deg,"
            + "launcher_bearing_deg,target_tps,feed_tps,feed_rpm,lowest_tps,in_zone,camera_pitch_deg";

    // ticks/s per D-pad right/left press (up/down = 4 presses).
    public static double SPEED_STEP = 25;
    // "Results" groups shots whose distances round to the same RANGE_BIN_IN inches.
    public static double RANGE_BIN_IN = 2;
    // A CELL seen this recently still counts when the ball is fed (the camera can lose it for a frame).
    public static double CELL_HOLD_SECONDS = 0.5;
    // How long after feeding stops the dip is still measured.
    public static double DIP_WINDOW_SECONDS = 0.6;
    // Only the camera and the launcher run in this test. Turn these on in FTC Dashboard BEFORE INIT
    // to drive (and aim with the left bumper) or run the intake; off = place the robot by hand.
    public static boolean USE_DRIVE = false;
    public static boolean USE_INTAKE = false;

    private MecanumDriveSubsystem drive = null;
    private IntakeSubsystem intake = null;
    private LauncherSubsystem launcher = null;
    private AprilTagVision vision = null;
    private final File file = new File(AppUtil.ROOT_FOLDER, LOG_PATH);
    private String fileProblem = null;
    private boolean savedDistanceSpeed;

    // The last CELL seen and when, so a one-frame dropout doesn't lose the distance.
    private AprilTagClusterDetection lastCell = null;
    private long lastCellMs = 0;

    // One shot = one hold of A: captured when the feed starts, written when marked.
    private boolean wasFeeding = false;
    private long lastFeedMs = 0;
    private Shot pending = null;
    private int shotCount = 0;
    // hits and shots per "range @ speed" combination, in the order they were tried.
    private final Map<String, int[]> results = new LinkedHashMap<>();
    private String message = "";

    private static class Shot {
        long timeMs;
        int number;
        String cell;
        double range, bearing, yaw, launcherBearing, target, feedTps, feedRpm, lowest, cameraPitch;
        boolean inZone;

        String key() {
            double bin = RANGE_BIN_IN > 0 ? Math.round(range / RANGE_BIN_IN) * RANGE_BIN_IN : range;
            return String.format(Locale.US, "%.0f in @ %.0f", bin, target);
        }
    }

    @Override
    public void init() {
        // Test OpModes always send to FTC Dashboard: they never run in a match.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = USE_DRIVE ? new MecanumDriveSubsystem(hardwareMap) : null;
        intake = USE_INTAKE ? new IntakeSubsystem(hardwareMap) : null;
        launcher = new LauncherSubsystem(hardwareMap);
        vision = new AprilTagVision(hardwareMap);
        launcher.setContinuousFeed(true);
        // One fixed speed per combination: the distance speed would change it as the robot moves.
        savedDistanceSpeed = LauncherSubsystem.USE_DISTANCE_SPEED;
        LauncherSubsystem.USE_DISTANCE_SPEED = false;
        telemetry.addData("Launcher", launcher.isEnabled() ? "on" : "OFF: set RobotSwitches.USE_LAUNCHER = true");
        telemetry.addData("Camera hardware", vision.hardwareStatus());
        telemetry.addData("Log", file.getPath());
        telemetry.addLine("Right bumper = launcher, hold A = feed nonstop, Y = hit, B = miss.");
    }

    @Override
    public void start() {
        if (drive != null) {
            drive.startTeleop();
            drive.toggleSlowMode();   // slow mode on: easier to stop at an exact distance
        }
    }

    @Override
    public void loop() {
        if (drive != null) {
            drive.update();
        }

        // ---------------- Camera ----------------
        AprilTagClusterDetection seen = vision.aimedUpCell();
        long nowMs = System.currentTimeMillis();
        if (seen != null) {
            lastCell = seen;
            lastCellMs = nowMs;
        }
        AprilTagClusterDetection cell = lastCell != null && nowMs - lastCellMs <= CELL_HOLD_SECONDS * 1000 ? lastCell : null;
        launcher.setCell(seen);   // for "in zone"

        // ---------------- Drive and intake (only when switched on above) ----------------
        if (drive != null) {
            if (gamepad1.xWasPressed()) {
                drive.toggleSlowMode();
            }
            if (gamepad1.left_bumper && cell != null) {
                drive.driveWithSticks(gamepad1, vision.aimTurn(vision.launcherBearing(cell)));
            } else {
                drive.driveWithSticks(gamepad1);
            }
        }
        if (intake != null) {
            intake.runTeleop(gamepad1.right_trigger - gamepad1.left_trigger, launcher.isFeeding());
        }

        // ---------------- Speed ----------------
        if (gamepad1.rightBumperWasPressed()) {
            launcher.toggleOn();
        }
        if (gamepad1.dpadRightWasPressed()) {
            changeSpeed(1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            changeSpeed(-1);
        }
        if (gamepad1.dpadUpWasPressed()) {
            changeSpeed(4);
        }
        if (gamepad1.dpadDownWasPressed()) {
            changeSpeed(-4);
        }

        // ---------------- One shot per hold of A ----------------
        launcher.run(gamepad1.a);            // once at speed, the roller runs until A is released
        boolean feeding = launcher.isFeeding();
        if (feeding && !wasFeeding) {
            startShot(cell);                 // the feed starts now: this is the shot's speed and distance
        }
        wasFeeding = feeding;
        if (feeding) {
            lastFeedMs = nowMs;
        }
        if (pending != null && nowMs - lastFeedMs <= DIP_WINDOW_SECONDS * 1000) {
            pending.lowest = Math.min(pending.lowest, launcher.getVelocity());
        }

        // ---------------- Hit / miss ----------------
        if (gamepad1.yWasPressed()) {
            mark("HIT");
        }
        if (gamepad1.bWasPressed()) {
            mark("MISS");
        }

        showTelemetry(seen);
    }

    private void changeSpeed(int steps) {
        // adjustTrim() moves by TRIM_STEP; use SPEED_STEP here without changing the TeleOp's trim step.
        double saved = LauncherSubsystem.TRIM_STEP;
        LauncherSubsystem.TRIM_STEP = SPEED_STEP;
        launcher.adjustTrim(steps);
        LauncherSubsystem.TRIM_STEP = saved;
    }

    private void startShot(AprilTagClusterDetection cell) {
        if (pending != null) {
            write(pending, "UNMARKED");      // the last shot was never marked
        }
        Shot s = new Shot();
        s.timeMs = System.currentTimeMillis();
        s.number = ++shotCount;
        boolean seen = cell != null && cell.ftcPose != null && cell.metadata != null;
        s.cell = seen ? cell.metadata.shortName : "none";
        s.range = seen ? cell.ftcPose.range : 0;
        s.bearing = seen ? cell.ftcPose.bearing : 0;
        s.yaw = seen ? cell.ftcPose.yaw : 0;
        s.launcherBearing = seen ? vision.launcherBearing(cell) : 0;
        s.target = launcher.getShootTarget();
        s.feedTps = launcher.getVelocity();
        s.feedRpm = launcher.getRpm();
        s.lowest = s.feedTps;
        s.inZone = seen && launcher.isInShootingZone();
        s.cameraPitch = AprilTagVision.CAMERA_PITCH_DEG;
        pending = s;
        message = String.format(Locale.US, "Shot %d: %s. Y = hit, B = miss", s.number,
                seen ? String.format(Locale.US, "%.1f in", s.range) : "NO CELL in view");
    }

    private void mark(String result) {
        if (pending == null) {
            message = "No shot to mark: hold A first";
            return;
        }
        Shot s = pending;
        write(s, result);
        pending = null;
        if (!"none".equals(s.cell)) {
            int[] hs = results.get(s.key());
            if (hs == null) {
                hs = new int[2];
                results.put(s.key(), hs);
            }
            if ("HIT".equals(result)) {
                hs[0]++;
            }
            hs[1]++;
        }
        message = String.format(Locale.US, "Shot %d saved: %s", s.number, result);
    }

    private void write(Shot s, String result) {
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.exists() && !dir.mkdirs()) {
                fileProblem = "can't create " + dir;
                return;
            }
            boolean isNew = !file.exists();
            try (FileWriter w = new FileWriter(file, true)) {
                if (isNew) {
                    w.write(HEADER + "\n");
                }
                w.write(String.format(Locale.US, "%d,%d,%s,%s,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%s,%.1f\n",
                        s.timeMs, s.number, result, s.cell, s.range, s.bearing, s.yaw, s.launcherBearing,
                        s.target, s.feedTps, s.feedRpm, s.lowest, s.inZone, s.cameraPitch));
            }
            fileProblem = null;
        } catch (IOException e) {
            fileProblem = "log file: " + e.getMessage();
        }
    }

    private void showTelemetry(AprilTagClusterDetection seen) {
        if (seen == null) {
            telemetry.addData("Camera", "no up CELL in view: turn toward the HIVE (left bumper aims)");
        } else {
            double lb = vision.launcherBearing(seen);
            telemetry.addData("Camera", "%s, %.1f in, launcher %.1f° %s, yaw %.1f°, %s",
                    seen.metadata.shortName, seen.ftcPose.range, Math.abs(lb), lb > 0 ? "left" : "right",
                    seen.ftcPose.yaw, vision.isAimedAt(seen) ? "AIMED" : "not aimed");
        }
        double target = launcher.getShootTarget();
        telemetry.addData("Speed", "%.0f of %.0f ticks/s (base %.0f %+.0f), %s",
                launcher.getVelocity(), target, LauncherSubsystem.TARGET_VELOCITY * LauncherSubsystem.SPEED_SCALE,
                launcher.getTrim(), !launcher.isOn() ? "OFF (right bumper)"
                        : launcher.isFeeding() ? "FEEDING..." : launcher.isAtSpeed() ? "READY: hold A" : "spinning up");
        telemetry.addData("Roller", launcher.getRollerText());
        telemetry.addData("Drive", drive != null ? drive.getModeText() : "off: place the robot by hand (USE_DRIVE)");
        telemetry.addData("Shot", message.isEmpty() ? "-" : message);

        telemetry.addData("Shots logged", "%d (%s)", shotCount, fileProblem != null ? fileProblem : LOG_PATH);
        telemetry.addLine("Results (hits / shots):");
        List<String> keys = new ArrayList<>(results.keySet());
        for (int i = Math.max(0, keys.size() - 10); i < keys.size(); i++) {   // the last 10 combinations
            int[] hs = results.get(keys.get(i));
            telemetry.addData("   " + keys.get(i), "%d / %d", hs[0], hs[1]);
        }
        launcher.addTelemetry(telemetry);   // Dashboard graph: "launcher velocity", "launcher target"
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pending != null) {
            write(pending, "UNMARKED");
            pending = null;
        }
        LauncherSubsystem.USE_DISTANCE_SPEED = savedDistanceSpeed;
        if (launcher != null) {
            launcher.stop();
        }
        if (vision != null) {
            vision.close();
        }
    }
}
