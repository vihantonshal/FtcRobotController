package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * LAUNCH ASSIST: remembers where the robot was when a launch scored, and helps the driver get back there.
 *
 *  - Every press of the operator's A (feed) with the launcher on, recordLaunch() writes one LAUNCH
 *    line to the log file.
 *  - When a ball scores, the operator presses B: markSuccess() saves the last launch as a SUCCESS,
 *    once per launch, and only if the camera saw a HIVE cluster then (otherwise check() could never
 *    use it). Successes are loaded again at every INIT, so they add up over practice sessions.
 *  - check() compares the robot now with the nearest saved success for the CELL the camera sees:
 *    is the pose OK to launch, and which way should it move? While the driver holds the align
 *    button, the TeleOp drives with check()'s forward/strafe/rotate.
 *
 * A launch pose is what the camera sees: the HIVE cluster (SDK 12 groups each HIVE face's four
 * tags, e.g. "RED SCORING") and the distance (range), angle (bearing) and turn (yaw) to its CELL
 * opening. It is measured from the CELL itself, so it doesn't matter where TeleOp started or how
 * many trips the robot made. With no up CELL in view, or none saved for the CELL in view, there is
 * no target: the LED stays blue and the align button does nothing, so the driver turns toward the
 * HIVE until the camera sees it.
 * The log also keeps the odometry pose (x, y, heading from INIT) for looking at later; check()
 * does not use it, because it depends on where TeleOp started and drifts with every trip.
 *
 * The log file is /sdcard/FIRST/BioBuzz/launch_log.csv on the Control Hub. Download it with
 * REV Hardware Client (or adb pull) and open it in a spreadsheet. Delete it to start fresh.
 *
 * BUILT-IN POSES: TeamCode/src/main/resources/launch_poses_default.csv is part of the app, so it is
 * deployed with the code and is on every Control Hub the code goes to. Same columns as the log file;
 * its SUCCESS lines are loaded first at every INIT, then the hub's log. The robot never changes it.
 * To add one: copy a SUCCESS line from the log, or the "Last launch CSV" line on FTC Dashboard,
 * into that file and deploy the code.
 * The tank-drive robot (BioBuzzStarterbotTeleopBase) has its own camera, so its own files:
 * launch_poses_tank.csv and BioBuzz/launch_log_tank.csv (new LaunchAssist(TANK_DEFAULTS_RESOURCE,
 * TANK_LOG_PATH)). It can't strafe, so it uses check(now, false).
 */
@Config
public class LaunchAssist {
    // ---- Tuning numbers (owner: launch team) ----
    // "Pose OK" when every error is inside these limits.
    public static double TAG_RANGE_TOL_IN = 2;
    public static double TAG_BEARING_TOL_DEG = 2;
    public static double TAG_YAW_TOL_DEG = 4;
    // Auto-align: drive power per inch or degree of error, and the most power it may use.
    public static double ALIGN_KP_DIST = 0.05;
    public static double ALIGN_KP_TURN = 0.015;
    public static double ALIGN_KP_YAW = 0.02;
    public static double ALIGN_MAX_POWER = 0.4;
    // In tag mode, a yaw error is fixed by strafing. If the robot strafes the wrong way, set -1.
    public static double YAW_STRAFE_SIGN = 1;

    public static final String LOG_PATH = "BioBuzz/launch_log.csv";
    // The built-in poses inside the app (TeamCode/src/main/resources), see the top of this file.
    public static final String DEFAULTS_RESOURCE = "/launch_poses_default.csv";
    // The tank-drive robot's own files (BioBuzzStarterbotTeleopBase): its camera sits differently, so
    // its poses must never mix with the mecanum robot's.
    public static final String TANK_LOG_PATH = "BioBuzz/launch_log_tank.csv";
    public static final String TANK_DEFAULTS_RESOURCE = "/launch_poses_tank.csv";
    private static final String HEADER = "time_ms,type,x_in,y_in,heading_deg,hive_cluster,tag_range_in,tag_bearing_deg,tag_yaw_deg,launcher_tps,launcher_rpm,flywheel_dir";
    private static final String NO_TAG = "none";

    // Where the robot is and what the launcher is doing, at one moment.
    public static class Snapshot {
        public final long timeMs;
        public final double x, y, headingDeg;
        public final String tag;                    // HIVE cluster name, e.g. "RED SCORING"; null when none is seen
        public final double tagRange, tagBearing, tagYaw;
        public final double launcherTps, launcherRpm;
        public final String flywheelDirection;

        public Snapshot(long timeMs, double x, double y, double headingDeg, String tag, double tagRange,
                        double tagBearing, double tagYaw, double launcherTps, double launcherRpm, String flywheelDirection) {
            this.timeMs = timeMs;
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
            this.tag = tag;
            this.tagRange = tagRange;
            this.tagBearing = tagBearing;
            this.tagYaw = tagYaw;
            this.launcherTps = launcherTps;
            this.launcherRpm = launcherRpm;
            this.flywheelDirection = flywheelDirection;
        }

        public boolean hasTag() {
            return tag != null;
        }

        // True when both snapshots see the same HIVE cluster.
        public boolean sameTag(Snapshot other) {
            return hasTag() && other.hasTag() && tag.equals(other.tag);
        }
    }

    // The result of check(): is the pose OK, a line for the driver, and the auto-align command.
    public static class Status {
        public boolean hasTarget = false;
        public boolean ok = false;
        public boolean tagMode = false;
        public String text = "no saved launch poses yet (operator B saves a good shot)";
        public double forward = 0, strafe = 0, rotate = 0;
    }

    // Switched on and off with RobotSwitches.USE_LAUNCH_ASSIST. When it is off, nothing is read or
    // written, check() never gives a target, and markSuccess() returns OFF.
    private final boolean enabled = RobotSwitches.USE_LAUNCH_ASSIST;
    // The built-in poses come first in this list, then the ones from the hub's log file.
    private final List<Snapshot> successes = new ArrayList<>();
    private int defaultCount = 0;
    private final String defaultsResource;
    private final File file;
    private Snapshot lastLaunch = null;
    private boolean lastLaunchSaved = false;   // B was already pressed for lastLaunch
    private String fileProblem = null;

    // TeleOp (update() keeps these): the robot now, the last snapshot that saw a CELL, this loop's
    // result, the gamepad LED color, the align rumble, and the "Saved ..." message.
    // A launch whose camera frame missed the CELL uses the last snapshot that saw one, if it is this recent.
    private static final double TAG_HOLD_SECONDS = 0.5;
    private Snapshot now = null;
    private Snapshot lastTagged = null;
    private Status status = new Status();
    private int ledState = -1;
    private boolean alignWasOk = false;
    private String message = null;
    private final ElapsedTime messageTimer = new ElapsedTime();

    // The mecanum robot's files: launch_poses_default.csv and /sdcard/FIRST/BioBuzz/launch_log.csv.
    public LaunchAssist() {
        this(DEFAULTS_RESOURCE, LOG_PATH);
    }

    /*
     * A robot with its own files: defaultsResource is the built-in poses file in
     * TeamCode/src/main/resources (e.g. "/launch_poses_tank.csv"), logPath the hub log under
     * /sdcard/FIRST (e.g. "BioBuzz/launch_log_tank.csv").
     */
    public LaunchAssist(String defaultsResource, String logPath) {
        this.defaultsResource = defaultsResource;
        file = new File(AppUtil.ROOT_FOLDER, logPath);
        if (enabled) {
            loadDefaults();
            loadSuccesses();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    // ---------------- TeleOp ----------------

    /*
     * Call once per loop: where the robot is now, compared with the nearest good launch pose for
     * the up CELL in view (cell may be null). The result says if the pose is OK and how to drive there.
     */
    public Status update(MecanumDriveSubsystem drive, AprilTagClusterDetection cell, LauncherSubsystem launcher) {
        now = snapshot(drive, cell, launcher);
        if (now.hasTag()) {
            lastTagged = now;
        }
        status = check(now);
        return status;
    }

    /*
     * The operator pressed A with the launcher on: one launch. (Feeding stops and starts whenever a shot
     * slows the flywheel, so it can't count launches.) If the camera missed the CELL in this one frame,
     * use the last snapshot that saw it, when it is at most TAG_HOLD_SECONDS old: the robot stands still
     * while it launches.
     */
    public void recordLaunch() {
        if (now == null) {
            return;
        }
        boolean recentTag = lastTagged != null && now.timeMs - lastTagged.timeMs <= TAG_HOLD_SECONDS * 1000;
        recordLaunch(!now.hasTag() && recentTag ? lastTagged : now);
    }

    // The operator pressed B, "that launch scored": save it, show what happened for 2 s, rumble when saved.
    public void saveLastLaunch(Gamepad operator) {
        switch (markSuccess()) {
            case SAVED:
                message = "Saved good launch #" + successCount();
                operator.rumble(150);
                break;
            case NO_TAG:
                message = "Not saved: the camera saw no CELL at that launch";
                break;
            case ALREADY_SAVED:
                message = "That launch is already saved";
                break;
            case OFF:
                message = "Launch assist is off (RobotSwitches)";
                break;
            default:
                message = "Nothing to save yet: launch first, then press B";
                break;
        }
        messageTimer.reset();
    }

    /*
     * Driver's gamepad LED: green = pose OK to launch, red = not yet, purple = up CELL in view but no
     * saved pose for it, blue = no up CELL in view. With launch assist switched off, the LED is left alone.
     */
    public void showOnLed(Gamepad driver, boolean cellInView) {
        if (!enabled) {
            return;
        }
        int state = status.hasTarget ? (status.ok ? 1 : 2) : (cellInView ? 3 : 0);
        if (state == ledState) {
            return;   // only send a new color when it changes
        }
        ledState = state;
        if (state == 1) {
            driver.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (state == 2) {
            driver.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (state == 3) {
            driver.setLedColor(1, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            driver.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    // While auto-aligning: rumble the driver's gamepad once when the robot gets there ("you're there").
    public void rumbleWhenAligned(Gamepad driver, boolean aligning) {
        boolean there = aligning && status.ok;
        if (there && !alignWasOk) {
            driver.rumble(200);
        }
        alignWasOk = there;
    }

    // The launch-assist lines the drive team always sees.
    public void addDriverLines(Telemetry telemetry, boolean aligning) {
        telemetry.addData("Launch pose", (aligning ? "ALIGNING: " : "") + status.text);
        telemetry.addData("Saved launch poses", "%d, %d built-in (operator B saves the last launch)",
                successCount(), defaultCount);
        if (message != null && messageTimer.seconds() < 2) {
            telemetry.addLine(message);
        }
        if (fileProblem != null) {
            telemetry.addData("Launch log", fileProblem);
        }
    }

    // ---------------- Snapshots, the log, and saved poses ----------------

    // What the robot is doing right now. tag (from AprilTagVision.closestTag()) may be null.
    public Snapshot snapshot(MecanumDriveSubsystem drive, AprilTagClusterDetection tag, LauncherSubsystem launcher) {
        return snapshot(drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg(), tag, launcher);
    }

    // The same, for a robot without MecanumDriveSubsystem: x, y, heading only go into the log.
    public Snapshot snapshot(double x, double y, double headingDeg, AprilTagClusterDetection tag, LauncherSubsystem launcher) {
        boolean seen = tag != null && tag.ftcPose != null && tag.metadata != null;
        return new Snapshot(System.currentTimeMillis(), x, y, headingDeg,
                seen ? tag.metadata.shortName : null, seen ? tag.ftcPose.range : 0, seen ? tag.ftcPose.bearing : 0, seen ? tag.ftcPose.yaw : 0,
                launcher.getVelocity(), launcher.getRpm(), launcher.getDirection());
    }

    // Call once per launch (the operator's A press): one LAUNCH line in the log.
    public void recordLaunch(Snapshot s) {
        if (!enabled) {
            return;
        }
        lastLaunch = s;
        lastLaunchSaved = false;
        append("LAUNCH", s);
    }

    // What markSuccess() did. Only SAVED adds a launch pose.
    public enum SaveResult { SAVED, NO_LAUNCH, NO_TAG, ALREADY_SAVED, OFF }

    /*
     * Call when the operator says the last launch scored. A launch with no HIVE cluster in view can
     * never be matched by check(), so it is not saved; neither is the same launch twice.
     */
    public SaveResult markSuccess() {
        if (!enabled) {
            return SaveResult.OFF;
        }
        if (lastLaunch == null) {
            return SaveResult.NO_LAUNCH;
        }
        if (lastLaunchSaved) {
            return SaveResult.ALREADY_SAVED;
        }
        if (!lastLaunch.hasTag()) {
            return SaveResult.NO_TAG;
        }
        successes.add(lastLaunch);
        lastLaunchSaved = true;
        append("SUCCESS", lastLaunch);
        return SaveResult.SAVED;
    }

    // How many saved launch poses check() can use as a target, built-in ones included.
    public int successCount() {
        return successes.size();
    }

    // How many of those came from the built-in file.
    public int defaultCount() {
        return defaultCount;
    }

    /*
     * For FTC Dashboard: every good launch pose check() can use (built-in ones first), and the last
     * launch as a SUCCESS line, ready to paste into the built-in file.
     */
    public void addTelemetry(Telemetry telemetry) {
        if (!enabled) {
            return;
        }
        telemetry.addData("Good launch poses", "%d built-in + %d saved on this hub",
                defaultCount, successes.size() - defaultCount);
        for (int i = 0; i < successes.size(); i++) {
            Snapshot s = successes.get(i);
            telemetry.addData(String.format(Locale.US, "Pose %d%s", i + 1, i < defaultCount ? " (built-in)" : ""),
                    "%s, %.1f in, bearing %.1f°, yaw %.1f°, flywheel %.0f ticks/s",
                    s.tag, s.tagRange, s.tagBearing, s.tagYaw, s.launcherTps);
        }
        if (lastLaunch == null) {
            telemetry.addData("Last launch CSV", "no launch yet");
        } else if (!lastLaunch.hasTag()) {
            telemetry.addData("Last launch CSV", "the camera saw no CELL, so it can't be a launch pose");
        } else {
            telemetry.addData("Last launch CSV", csvRow("SUCCESS", lastLaunch));
        }
    }

    public Snapshot getLastLaunch() {
        return lastLaunch;
    }

    // null when the log file works; otherwise what went wrong, for telemetry.
    public String getFileProblem() {
        return fileProblem;
    }

    /*
     * Compare "now" with the nearest saved success for the same HIVE CELL: match the distance,
     * bearing and yaw to its CELL opening. No up CELL in view, or none saved for this CELL: no target.
     */
    public Status check(Snapshot now) {
        return check(now, true);
    }

    /*
     * canStrafe = false for a tank (left/right) drive: it can't strafe, so the yaw error is not fixed
     * (strafe stays 0) and "OK" only needs distance and bearing.
     */
    public Status check(Snapshot now, boolean canStrafe) {
        Status st = new Status();
        if (!enabled) {
            st.text = "launch assist off (RobotSwitches)";
            return st;
        }
        if (successes.isEmpty()) {
            return st;   // "no saved launch poses yet"
        }
        if (!now.hasTag()) {
            st.text = "no up CELL in view: turn toward the HIVE";
            return st;
        }
        Snapshot target = nearest(now);
        if (target == null) {
            st.text = String.format(Locale.US, "no saved pose for %s yet (operator B saves a good shot)", now.tag);
            return st;
        }
        st.hasTarget = true;
        st.tagMode = true;
        double eRange = now.tagRange - target.tagRange;         // + = too far away
        double eBearing = now.tagBearing - target.tagBearing;   // + = tag too far left
        double eYaw = now.tagYaw - target.tagYaw;
        st.ok = Math.abs(eRange) < TAG_RANGE_TOL_IN && Math.abs(eBearing) < TAG_BEARING_TOL_DEG
                && (!canStrafe || Math.abs(eYaw) < TAG_YAW_TOL_DEG);
        st.forward = cap(ALIGN_KP_DIST * eRange);
        st.rotate = cap(-ALIGN_KP_TURN * eBearing);   // + rotate turns right
        st.strafe = canStrafe ? cap(YAW_STRAFE_SIGN * ALIGN_KP_YAW * eYaw) : 0;
        String move = String.format(Locale.US, "%s: %s %.1f in, turn %.0f° %s", now.tag,
                eRange > 0 ? "move in" : "back up", Math.abs(eRange), Math.abs(eBearing), eBearing > 0 ? "left" : "right");
        st.text = st.ok
                ? String.format(Locale.US, "OK (%s)", now.tag)
                : canStrafe ? move + String.format(Locale.US, ", angle off %.0f°", eYaw) : move;
        return st;
    }

    // The saved success for the same HIVE CELL that is closest in distance and angle, or null.
    private Snapshot nearest(Snapshot now) {
        Snapshot best = null;
        double bestScore = Double.MAX_VALUE;
        for (Snapshot s : successes) {
            if (!now.sameTag(s)) {
                continue;
            }
            double score = Math.abs(now.tagRange - s.tagRange) + 0.2 * Math.abs(now.tagBearing - s.tagBearing);
            if (score < bestScore) {
                bestScore = score;
                best = s;
            }
        }
        return best;
    }

    private static double cap(double v) {
        return Range.clip(v, -ALIGN_MAX_POWER, ALIGN_MAX_POWER);
    }

    // ---- The log file ----

    private void append(String type, Snapshot s) {
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
                w.write(csvRow(type, s) + "\n");
            }
            fileProblem = null;
        } catch (IOException e) {
            fileProblem = "log file: " + e.getMessage();
        }
    }

    // One line of the log file (also the built-in file's format), without the line break.
    public static String csvRow(String type, Snapshot s) {
        return String.format(Locale.US, "%d,%s,%.2f,%.2f,%.1f,%s,%.2f,%.2f,%.2f,%.0f,%.0f,%s",
                s.timeMs, type, s.x, s.y, s.headingDeg, s.hasTag() ? s.tag : NO_TAG, s.tagRange, s.tagBearing, s.tagYaw,
                s.launcherTps, s.launcherRpm, s.flywheelDirection);
    }

    // The built-in poses from inside the app. No file in this build: no built-in poses.
    private void loadDefaults() {
        InputStream in = LaunchAssist.class.getResourceAsStream(defaultsResource);
        if (in == null) {
            return;
        }
        try (BufferedReader r = new BufferedReader(new InputStreamReader(in))) {
            readSuccesses(r);
        } catch (IOException e) {
            fileProblem = "built-in poses: " + e.getMessage();
        }
        defaultCount = successes.size();
    }

    private void loadSuccesses() {
        if (!file.exists()) {
            return;
        }
        try (BufferedReader r = new BufferedReader(new FileReader(file))) {
            readSuccesses(r);
        } catch (IOException e) {
            fileProblem = "log file: " + e.getMessage();
        }
    }

    // Adds every usable SUCCESS line to successes. Other lines (header, LAUNCH, # notes) are skipped.
    private void readSuccesses(BufferedReader r) throws IOException {
        String line;
        while ((line = r.readLine()) != null) {
            if (line.trim().startsWith("#")) {
                continue;   // a note, or a line switched off
            }
            String[] c = line.split(",");
            if (c.length < 12) {
                continue;
            }
            // The built-in file is typed by hand, so allow spaces around each value.
            for (int i = 0; i < c.length; i++) {
                c[i] = c[i].trim();
            }
            if (!"SUCCESS".equals(c[1])) {
                continue;
            }
            // Only rows with a HIVE cluster name can be a target. Skip "none" (or "-1" in old logs),
            // and old logs' tag numbers like "31", which never equal a cluster name.
            String tag = c[5];
            if (tag.isEmpty() || NO_TAG.equals(tag) || tag.matches("-?\\d+")) {
                continue;
            }
            try {
                successes.add(new Snapshot(Long.parseLong(c[0]), Double.parseDouble(c[2]), Double.parseDouble(c[3]),
                        Double.parseDouble(c[4]), tag, Double.parseDouble(c[6]), Double.parseDouble(c[7]),
                        Double.parseDouble(c[8]), Double.parseDouble(c[9]), Double.parseDouble(c[10]), c[11]));
            } catch (NumberFormatException ignored) {
                // a damaged line: skip it
            }
        }
    }
}
