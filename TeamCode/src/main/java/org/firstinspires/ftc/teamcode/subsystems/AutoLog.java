package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/*
 * The Auto's notebook: what happened, step by step, so the drive team can read the match afterwards.
 *
 * STEP LOG: every step change (and START and STOP) is one row in /sdcard/FIRST/BioBuzz/auto_log.csv
 * on the Control Hub, with the reason, the time since START and where odometry says the robot is.
 * Download it after a match:  adb pull /sdcard/FIRST/BioBuzz/auto_log.csv
 * The same steps also stay on the Driver Hub (the "Step 1", "Step 2"... lines) until the next INIT.
 * "run" is the date and time START was pressed, so the rows of one match belong together.
 *
 * TAG LOG: did the camera see the HIVE's AprilTags, and which? One row in
 * /sdcard/FIRST/BioBuzz/auto_tags.csv each time what the camera sees changes (another CELL, more or
 * fewer of its tags, UP or down, or nothing), and at most once a second while it stays the same.
 * Example row: run, 2.97, AIM, RED AUDIENCE IDs 34-37 2/4 tags UP 35 in 2 deg left
 * SDK 12 reports a HIVE cluster's 4 tags together, so a row shows the cluster's ID range and how
 * many of its tags were found, not single tag numbers. "none" = no tag seen.
 *
 * It also keeps the match clock (secondsSinceStart()) and where the robot launched from.
 */
public class AutoLog {
    public static final String LOG_PATH = "BioBuzz/auto_log.csv";
    private static final String LOG_HEADER = "run,t_s,from,to,reason,x_in,y_in,heading_deg,up_cell,launcher_tps";
    public static final String TAG_LOG_PATH = "BioBuzz/auto_tags.csv";
    private static final String TAG_LOG_HEADER = "run,t_s,state,camera_sees";

    private final MecanumDriveSubsystem drive;
    private final AprilTagVision vision;
    private final LauncherSubsystem launcher;
    private final File logFile = new File(AppUtil.ROOT_FOLDER, LOG_PATH);
    private final File tagLogFile = new File(AppUtil.ROOT_FOLDER, TAG_LOG_PATH);
    private final ElapsedTime clock = new ElapsedTime();       // from START
    private final ElapsedTime tagLogTimer = new ElapsedTime();
    private final List<String> steps = new ArrayList<>();
    private String runId = "";
    private String problem = null;          // shown on the Driver Hub if a file can't be written
    private String lastSeenKey = null;      // what was seen at the last tag-log row (without distances)
    private String cameraSees = "";         // the Driver Hub's "Camera sees" line
    private String launchSpot = null;       // the Driver Hub's "Launched" line

    // vision may be null, for an Auto without a camera (BioBuzzStarterBotAutoNoCamera).
    public AutoLog(MecanumDriveSubsystem drive, AprilTagVision vision, LauncherSubsystem launcher) {
        this.drive = drive;
        this.vision = vision;
        this.launcher = launcher;
    }

    // Call at START: the match clock starts, and "run" is the date and time now.
    public void start() {
        clock.reset();
        runId = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
    }

    // False until START: a STOP before START has nothing to log.
    public boolean isStarted() {
        return !runId.isEmpty();
    }

    public double secondsSinceStart() {
        return clock.seconds();
    }

    /*
     * One row in the step log and one line in the Driver Hub's step list. Called only when the step
     * changes (about 7 times a match), so writing the file does not slow the loop down.
     */
    public void step(String from, String to, String reason) {
        double t = clock.seconds();
        AprilTagClusterDetection cell = vision != null ? vision.closestTag() : null;
        String upCell = vision == null ? "no camera" : !vision.isEnabled() ? "camera off"
                : cell == null ? "none" : cell.metadata.shortName;
        steps.add(String.format(Locale.US, "%5.1f s  %s -> %s: %s", t, from, to, reason));
        String row = String.format(Locale.US, "%s,%.2f,%s,%s,%s,%.1f,%.1f,%.1f,%s,%.0f",
                runId, t, from, to, reason, drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg(),
                upCell, launcher.getVelocity());
        appendRow(logFile, LOG_HEADER, row, "step log");
    }

    /*
     * Call once per loop: one row in the tag log when what the camera sees changes, or once a second
     * while it stays the same, so the file shows when the camera saw the HIVE's tags and when it saw nothing.
     */
    public void tags(String state) {
        if (vision == null) {   // an Auto without a camera: no tag log
            cameraSees = "no camera";
            return;
        }
        String key = vision.seenSummary(false);
        cameraSees = vision.seenSummary(true);
        if (key.equals(lastSeenKey) && tagLogTimer.seconds() < 1) {
            return;
        }
        lastSeenKey = key;
        tagLogTimer.reset();
        appendRow(tagLogFile, TAG_LOG_HEADER, String.format(Locale.US, "%s,%.2f,%s,%s",
                runId, clock.seconds(), state, cameraSees), "tag log");
    }

    /*
     * Remembers where the robot launches from, for the "Launched" line: which CELL it aimed at, how far
     * it turned from its start heading (+ = left), and where odometry says it is. The cluster name gives
     * the HIVE end on both alliances: SCORING = far CELL, AUDIENCE = audience CELL.
     * Example: "RED SCORING (far CELL), turned 15° left, at (0.3, -0.1) in".
     */
    public void launchSpot(double startHeadingDeg) {
        AprilTagClusterDetection cell = vision != null ? vision.closestTag() : null;
        String target;
        if (vision == null) {
            target = "no camera (fixed launch spot)";
        } else if (!vision.isEnabled()) {
            target = "camera off";
        } else if (cell == null) {
            target = "no up CELL found";
        } else {
            String name = cell.metadata.shortName;
            target = name + (name.contains("SCORING") ? " (far CELL)" : " (audience CELL)");
        }
        double turned = AngleUnit.normalizeDegrees(drive.getPoseHeadingDeg() - startHeadingDeg);
        launchSpot = String.format(Locale.US, "%s, turned %.0f° %s, at (%.1f, %.1f) in", target,
                Math.abs(turned), turned >= 0 ? "left" : "right", drive.getPoseX(), drive.getPoseY());
    }

    public boolean hasLaunchSpot() {
        return launchSpot != null;
    }

    // What the camera sees, where the robot launched, and every step so far, one line each.
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera sees", cameraSees);
        if (launchSpot != null) {
            telemetry.addData("Launched", launchSpot);
        }
        for (int i = 0; i < steps.size(); i++) {
            telemetry.addData("Step " + (i + 1), steps.get(i));
        }
        if (problem != null) {
            telemetry.addData("Log NOT saved", problem);
        }
    }

    // Adds one line to a CSV file on the Control Hub (with the header if the file is new).
    private void appendRow(File file, String header, String row, String what) {
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.exists() && !dir.mkdirs()) {
                problem = "can't create " + dir;
                return;
            }
            boolean isNew = !file.exists();
            try (FileWriter w = new FileWriter(file, true)) {
                if (isNew) {
                    w.write(header + "\n");
                }
                w.write(row + "\n");
            }
            problem = null;
        } catch (IOException e) {
            problem = what + ": " + e.getMessage();
        }
    }
}
