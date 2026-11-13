package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Locale;

/*
 * CAMERA TEST: only the camera, nothing else on the robot is used (no motors, no servos).
 * Press INIT and point the webcam at the HIVE tags. No alliance and no START needed.
 *
 * What you should see:
 *   Driver Hub        "Camera hardware: found "Webcam 1", plugged in", "Camera: STREAMING ...",
 *                     then one line per CELL the camera sees.
 *   Monitor (HDMI)    the camera picture with the tags outlined (the SDK's live view).
 *   FTC Dashboard     http://192.168.43.1:8080/dash, layout Default: the picture in the left tile.
 *
 * It uses the same AprilTagVision as the TeleOp and the Auto, so what works here works there.
 * RobotSwitches.USE_CAMERA must be true (and USE_DASHBOARD true for the Dashboard picture).
 *
 * DASHBOARD
 *                     "CSV ..." lines: the up CELL's hive_cluster, tag_range_in, tag_bearing_deg and
 *                     tag_yaw_deg, exactly as launch_log.csv would get them.
 *   Graph:          "cell range", "cell bearing", "cell yaw" (inches, degrees; 0 when no CELL is in view).
 *   Configuration:  AprilTagVision   MANUAL_EXPOSURE, EXPOSURE_MS, GAIN (a dark or blurry picture),
 *                                    DECIMATION (how far tags are found), MAX_RESULT_AGE_MS,
 *                                    RANGE_AVERAGE_FRAMES (a steadier range), CAMERA_HEIGHT_IN,
 *                                    CAMERA_PITCH_DEG, UP_CELL_MIN_HEIGHT_IN (which CELL counts as up).
 *   Changes work at once. They are lost when the robot restarts: copy the keepers into AprilTagVision.java.
 *
 * PLAIN CAMERA (BioBuzzCameraTest.PLAIN_CAMERA in Dashboard, set it BEFORE pressing INIT): only the
 * webcam picture, exactly like the SDK's UtilityCameraFrameCapture: no AprilTag processor and no
 * exposure / gain settings. If the picture shows in PLAIN mode but not in normal mode, the problem is
 * the AprilTag processor or the camera settings, not the webcam.
 */
@Config
@TeleOp(name = "Camera Test", group = "StarterBot")
@Disabled
public class BioBuzzCameraTest extends OpMode {

    // true = picture only (see PLAIN CAMERA above). Read at INIT.
    public static boolean PLAIN_CAMERA = false;
    // CAMERA TILT FINDER: tape-measure the up CELL opening's center height above the TILES (inches) and
    // type it here in Dashboard. Camera Test then shows the camera tilt that makes "height" match it;
    // copy that into AprilTagVision.CAMERA_PITCH_DEG. 0 = off. Measure CAMERA_HEIGHT_IN (lens) first.
    public static double KNOWN_CELL_HEIGHT_IN = 0;

    private AprilTagVision vision = null;
    private boolean plain = false;

    @Override
    public void init() {
        // Test OpModes always send to FTC Dashboard: they never run in a match.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        plain = PLAIN_CAMERA;
        vision = new AprilTagVision(hardwareMap, true, plain);
        telemetry.addData("Mode", plain ? "PLAIN camera (picture only, no AprilTags)" : "AprilTags");
        telemetry.addData("Camera hardware", vision.hardwareStatus());
        telemetry.addLine("Point the camera at the HIVE tags. START is not needed.");
    }

    @Override
    public void init_loop() {
        showCamera();
    }

    @Override
    public void loop() {
        showCamera();
    }

    private void showCamera() {
        telemetry.addData("Mode", plain ? "PLAIN camera (picture only, no AprilTags)" : "AprilTags");
        telemetry.addData("Seen", vision.seenSummary(true));
        vision.addTelemetry(telemetry);   // hardware, camera state and fps, one line per CELL
        // The up CELL the robot points at: the one the TeleOp aims at and LaunchAssist writes to its CSV.
        AprilTagClusterDetection cell = vision.aimedUpCell();
        if (cell == null) {
            telemetry.addData("CSV tag values", "no up CELL in view (the CSV would get an empty cluster and 0, 0, 0)");
        } else {
            // The same columns and format as launch_log.csv (LaunchAssist): + bearing = left.
            telemetry.addData("CSV hive_cluster", cell.metadata.shortName);
            telemetry.addData("CSV tag_range_in", "%.2f", cell.ftcPose.range);
            telemetry.addData("CSV tag_bearing_deg", "%.2f", cell.ftcPose.bearing);
            telemetry.addData("CSV tag_yaw_deg", "%.2f", cell.ftcPose.yaw);
            telemetry.addData("CSV line part", String.format(Locale.US, "%s,%.2f,%.2f,%.2f",
                    cell.metadata.shortName, cell.ftcPose.range, cell.ftcPose.bearing, cell.ftcPose.yaw));
            double stand = vision.intakeToStandInches(cell);
            telemetry.addData("Intake to HIVE stand", "~%.1f in (%.1f ft) = range %.1f - AprilTagVision.STAND_OFFSET_IN %.1f; face the CELL square",
                    stand, stand / 12, cell.ftcPose.range, AprilTagVision.STAND_OFFSET_IN);
            showTilt(cell);
        }
        // Dashboard graph lines (0 = none).
        telemetry.addData("cell range", cell != null ? cell.ftcPose.range : 0);
        telemetry.addData("cell bearing", cell != null ? cell.ftcPose.bearing : 0);
        telemetry.addData("cell yaw", cell != null ? cell.ftcPose.yaw : 0);
        telemetry.addData("intake to stand", cell != null ? vision.intakeToStandInches(cell) : 0);
        telemetry.update();
    }

    /*
     * The camera sees the opening y in. ahead and z in. up IN ITS OWN PICTURE. Tilted up by p, the true
     * height above the lens is y*sin(p) + z*cos(p) (AprilTagVision.cellHeightInches). Knowing the real
     * height, solve for p: with r = hypot(y, z) and a = atan2(z, y), r*sin(p + a) = height - lens height.
     */
    private void showTilt(AprilTagClusterDetection cell) {
        double y = cell.ftcPose.y, z = cell.ftcPose.z;
        telemetry.addData("TILT camera sees", "%.1f in ahead, %.1f in up (in its picture); height %.1f in with tilt %.0f°",
                y, z, vision.cellHeightInches(cell), AprilTagVision.CAMERA_PITCH_DEG);
        if (KNOWN_CELL_HEIGHT_IN <= 0) {
            telemetry.addData("TILT finder", "off: tape-measure the opening height, set BioBuzzCameraTest.KNOWN_CELL_HEIGHT_IN");
            return;
        }
        double s = (KNOWN_CELL_HEIGHT_IN - AprilTagVision.CAMERA_HEIGHT_IN) / Math.hypot(y, z);
        if (Math.abs(s) > 1) {
            telemetry.addData("TILT finder", "no tilt fits: check KNOWN_CELL_HEIGHT_IN and CAMERA_HEIGHT_IN");
            return;
        }
        double tilt = Math.toDegrees(Math.asin(s) - Math.atan2(z, y));
        telemetry.addData("TILT finder", "camera tilt %.1f° up: set AprilTagVision.CAMERA_PITCH_DEG = %.1f", tilt, tilt);
        telemetry.addData("camera tilt", tilt);   // graph it: it should stay steady while you move
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }
}
