package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * LAUNCH ASSIST: remembers where the robot was when a launch scored, and helps the driver get back there.
 *
 *  - Every time the windmill starts feeding, recordLaunch() writes a LAUNCH line to the log file.
 *  - When a ball scores, the operator presses B: markSuccess() saves the last launch as a SUCCESS.
 *    Successes are loaded again at every INIT, so they add up over practice sessions.
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

    private final List<Snapshot> successes = new ArrayList<>();
    private final File file;
    private Snapshot lastLaunch = null;
    private String fileProblem = null;

    public LaunchAssist() {
        file = new File(AppUtil.ROOT_FOLDER, LOG_PATH);
        loadSuccesses();
    }

    // What the robot is doing right now. tag (from AprilTagVision.closestTag()) may be null.
    public Snapshot snapshot(MecanumDriveSubsystem drive, AprilTagClusterDetection tag, LauncherSubsystem launcher) {
        boolean seen = tag != null && tag.ftcPose != null && tag.metadata != null;
        return new Snapshot(System.currentTimeMillis(), drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg(),
                seen ? tag.metadata.shortName : null, seen ? tag.ftcPose.range : 0, seen ? tag.ftcPose.bearing : 0, seen ? tag.ftcPose.yaw : 0,
                launcher.getVelocity(), launcher.getRpm(), launcher.getDirection());
    }

    // Call when the windmill starts feeding: one LAUNCH line in the log.
    public void recordLaunch(Snapshot s) {
        lastLaunch = s;
        append("LAUNCH", s);
    }

    // Call when the operator says the last launch scored. Returns false if there was no launch yet.
    public boolean markSuccess() {
        if (lastLaunch == null) {
            return false;
        }
        successes.add(lastLaunch);
        append("SUCCESS", lastLaunch);
        return true;
    }

    public int successCount() {
        return successes.size();
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
        Status st = new Status();
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
                && Math.abs(eYaw) < TAG_YAW_TOL_DEG;
        st.forward = cap(ALIGN_KP_DIST * eRange);
        st.rotate = cap(-ALIGN_KP_TURN * eBearing);   // + rotate turns right
        st.strafe = cap(YAW_STRAFE_SIGN * ALIGN_KP_YAW * eYaw);
        st.text = st.ok
                ? String.format(Locale.US, "OK (%s)", now.tag)
                : String.format(Locale.US, "%s: %s %.1f in, turn %.0f° %s, angle off %.0f°", now.tag,
                eRange > 0 ? "move in" : "back up", Math.abs(eRange), Math.abs(eBearing), eBearing > 0 ? "left" : "right", eYaw);
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
                w.write(String.format(Locale.US, "%d,%s,%.2f,%.2f,%.1f,%s,%.2f,%.2f,%.2f,%.0f,%.0f,%s\n",
                        s.timeMs, type, s.x, s.y, s.headingDeg, s.hasTag() ? s.tag : NO_TAG, s.tagRange, s.tagBearing, s.tagYaw,
                        s.launcherTps, s.launcherRpm, s.flywheelDirection));
            }
            fileProblem = null;
        } catch (IOException e) {
            fileProblem = "log file: " + e.getMessage();
        }
    }

    private void loadSuccesses() {
        if (!file.exists()) {
            return;
        }
        try (BufferedReader r = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] c = line.split(",");
                if (c.length < 12 || !"SUCCESS".equals(c[1])) {
                    continue;
                }
                // "none" (or "-1" in old logs) = no tag. Old logs hold a tag number like "31", which never
                // equals a cluster name, so those rows are loaded but never used as a target.
                String tag = (c[5].isEmpty() || NO_TAG.equals(c[5]) || "-1".equals(c[5])) ? null : c[5];
                try {
                    successes.add(new Snapshot(Long.parseLong(c[0]), Double.parseDouble(c[2]), Double.parseDouble(c[3]),
                            Double.parseDouble(c[4]), tag, Double.parseDouble(c[6]), Double.parseDouble(c[7]),
                            Double.parseDouble(c[8]), Double.parseDouble(c[9]), Double.parseDouble(c[10]), c[11]));
                } catch (NumberFormatException ignored) {
                    // a damaged line: skip it
                }
            }
        } catch (IOException e) {
            fileProblem = "log file: " + e.getMessage();
        }
    }
}
