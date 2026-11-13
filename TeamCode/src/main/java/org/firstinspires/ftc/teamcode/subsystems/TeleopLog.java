package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/*
 * TELEOP LOG: what the TeleOp decided, for finding out afterwards why the robot did (or didn't) align
 * or feed. One row in /sdcard/FIRST/BioBuzz/teleop_log.csv each time something changes (the CELL in
 * view, the saved pose picked as the target, aligning on/off, pose OK, launcher on/off, A held,
 * feeding, or why A isn't feeding), and at most once every REPEAT_SECONDS while aligning or feeding.
 * Download it:  adb pull /sdcard/FIRST/BioBuzz/teleop_log.csv
 *
 * Columns:
 *   run, t_s                     which TeleOp run (start time in ms) and seconds since INIT
 *   cell, range_in, bearing_deg, yaw_deg              what the camera sees now (blank = no up CELL)
 *   target_pose, target_range_in, target_bearing_deg, target_yaw_deg
 *                                the saved pose LaunchAssist picked (number as in "Pose N" on FTC
 *                                Dashboard, 0 = none) and its values
 *   pose_ok, aligning            inside the tolerances?  D-pad up held with a target?
 *   fwd, strafe, rot             what auto-align sent to the wheels (0 when not aligning)
 *   launcher_on, flywheel_tps, target_tps, a_held, feeding
 *   feed_state                   "feeding", or WHY A isn't feeding ("launcher OFF", "waiting for speed")
 *   assist                       LaunchAssist's line for the driver
 *
 * The file stays open while the TeleOp runs and is written only when a row is due, so it adds no lag;
 * it is flushed every REPEAT_SECONDS and closed at STOP.
 */
@Config
public class TeleopLog {
    public static final String LOG_PATH = "BioBuzz/teleop_log.csv";
    public static double REPEAT_SECONDS = 1.0;
    private static final String HEADER = "run,t_s,cell,range_in,bearing_deg,yaw_deg,target_pose,target_range_in,"
            + "target_bearing_deg,target_yaw_deg,pose_ok,aligning,fwd,strafe,rot,launcher_on,flywheel_tps,"
            + "target_tps,a_held,feeding,feed_state,assist";

    private final long run = System.currentTimeMillis();
    private final ElapsedTime clock = new ElapsedTime();
    private BufferedWriter out;
    private String lastState = "";
    private double lastWriteS = -100, lastFlushS = 0;
    private String problem;

    public TeleopLog() {
        File file = new File(AppUtil.ROOT_FOLDER, LOG_PATH);
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.exists() && !dir.mkdirs()) {
                problem = "can't create " + dir;
                return;
            }
            boolean isNew = !file.exists();
            out = new BufferedWriter(new FileWriter(file, true));
            if (isNew) {
                out.write(HEADER + "\n");
            }
        } catch (IOException e) {
            problem = e.getMessage();
            out = null;
        }
    }

    // Once per loop, after the launcher and LaunchAssist have run.
    public void update(AprilTagClusterDetection cell, LaunchAssist.Status pose, boolean aligning,
                       LauncherSubsystem launcher, boolean aHeld) {
        if (out == null) {
            return;
        }
        double t = clock.seconds();
        String feedState = launcher.feedState(aHeld);
        // Only what matters for "did something change" (not the numbers that wobble every loop).
        String state = (cell == null ? "-" : cell.metadata.shortName) + "|" + pose.targetPose + "|" + pose.ok
                + "|" + aligning + "|" + launcher.isOn() + "|" + aHeld + "|" + launcher.isFeeding() + "|" + feedState;
        boolean busy = aligning || aHeld || launcher.isFeeding();
        if (state.equals(lastState) && !(busy && t - lastWriteS >= REPEAT_SECONDS)) {
            return;
        }
        lastState = state;
        lastWriteS = t;
        boolean seen = cell != null && cell.ftcPose != null;
        try {
            out.write(String.format(Locale.US,
                    "%d,%.2f,%s,%s,%s,%s,%d,%s,%s,%s,%b,%b,%.2f,%.2f,%.2f,%b,%.0f,%.0f,%b,%b,%s,%s%n",
                    run, t,
                    seen ? cell.metadata.shortName : "", num(seen, seen ? cell.ftcPose.range : 0),
                    num(seen, seen ? cell.ftcPose.bearing : 0), num(seen, seen ? cell.ftcPose.yaw : 0),
                    pose.targetPose, num(pose.hasTarget, pose.targetRange), num(pose.hasTarget, pose.targetBearing),
                    num(pose.hasTarget, pose.targetYaw), pose.ok, aligning,
                    aligning ? pose.forward : 0, aligning ? pose.strafe : 0, aligning ? pose.rotate : 0,
                    launcher.isOn(), launcher.getVelocity(), launcher.getShootTarget(), aHeld, launcher.isFeeding(),
                    clean(feedState), clean(pose.text)));
            if (t - lastFlushS >= REPEAT_SECONDS) {
                out.flush();
                lastFlushS = t;
            }
        } catch (IOException e) {
            problem = e.getMessage();
        }
    }

    // A file problem for the Driver Hub, or null.
    public String getProblem() {
        return problem;
    }

    // At STOP: write what is left and close the file.
    public void close() {
        if (out == null) {
            return;
        }
        try {
            out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    private static String num(boolean have, double v) {
        return have ? String.format(Locale.US, "%.1f", v) : "";
    }

    // No commas or line breaks inside a CSV cell.
    private static String clean(String s) {
        return s == null ? "" : s.replace(',', ';').replace('\n', ' ');
    }
}
