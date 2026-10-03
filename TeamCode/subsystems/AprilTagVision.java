package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/*
 * The webcam and the BIOBUZZ HIVE AprilTags, using SDK 12's tag clusters.
 * Switched on and off with RobotSwitches.USE_CAMERA; when it is off, closestTag() always
 * returns null.
 *
 * Each HIVE face has a CLUSTER of four tags (30-33 RED SCORING, 34-37 RED AUDIENCE,
 * 38-41 BLUE AUDIENCE, 42-45 BLUE SCORING). The SDK's BIOBUZZ library knows where each tag sits,
 * so it turns the tags it sees (one is enough, more is steadier) into ONE pose: the center of the
 * CELL opening. Distance (range) and angle (bearing) are measured to that opening, which is
 * exactly what we aim the launcher at. A tag in a cluster is only reported as part of its cluster.
 *
 *  - closestTag(): the UP CELL to launch into (metadata.shortName = its name), or null
 *  - cellHeightInches(): how high a CELL opening is above the TILES
 *  - aimTurn(bearing): the turn power that points the robot at it
 *  - streams the camera picture to FTC Dashboard (http://192.168.43.1:8080/dash)
 *
 * Which CELL is up: each HIVE is a see-saw with two CELLS (red HIVE: RED SCORING + RED AUDIENCE,
 * blue HIVE: BLUE SCORING + BLUE AUDIENCE), and the up CELL's opening is the higher one.
 *  - Both CELLS of a HIVE in view: the higher one is up.
 *  - Only one in view: it is up if it is higher than UP_CELL_MIN_HEIGHT_IN.
 * If up CELLS of both HIVES are in view, the nearer one wins. If only a down CELL is in view,
 * closestTag() returns null, so we never aim at the down CELL.
 *
 * Bearing is in degrees. Positive bearing means the CELL opening is to the robot's LEFT.
 * It does not check the alliance.
 *
 * LIMELIGHT 3A instead of the webcam: set RobotSwitches.USE_LIMELIGHT = true. Everything above stays
 * the same for the Auto and the TeleOp. The Limelight finds the tags itself (one result per tag, with
 * its real ID); this class groups them into CELLS with the same BIOBUZZ library (lookupCluster) and
 * makes the same kind of CELL result. Differences from the webcam:
 *  - the CELL pose is the AVERAGE of the CELL's tags the Limelight sees, not the SDK's exact CELL
 *    opening (a few inches apart): re-save the TeleOp's good launch poses after switching cameras;
 *  - the tag log shows the real tag IDs ("IDs 34 35"), not just the cluster's ID range;
 *  - no camera picture on FTC Dashboard; use the Limelight's own web page.
 * Setup when it arrives (see also StarterBot_Change_Guide.md):
 *  1. Driver Hub configuration: the Limelight shows up as a USB device; name it "limelight".
 *  2. Limelight web page, pipeline LIMELIGHT_PIPELINE (0): AprilTag, family 36h11, tag size as shown on
 *     the Driver Hub at INIT ("Limelight tag size"), and full 3D on, so each tag has a 3D pose.
 *  3. Measure the lens height and upward tilt again for CAMERA_HEIGHT_IN and CAMERA_PITCH_DEG.
 *  4. Check the "Camera sees" and UP/down lines facing each CELL before trusting the Auto.
 */
@Config
public class AprilTagVision {
    // ---- Tuning numbers (owner: vision team) ----
    /*
     * Aiming uses a "P" (proportional) rule: turn power = AIM_TURN_GAIN * angle. With 0.01, a CELL
     * opening 25 degrees away gives 0.25 turn power, and the robot slows down as it lines up.
     * AIM_MAX_TURN caps the turn power. From the SDK sample RobotAutoDriveToAprilTagOmni.
     */
    public static double AIM_TURN_GAIN = 0.01;
    public static double AIM_MAX_TURN = 0.3;
    public static double AIM_TOLERANCE_DEG = 2;

    /*
     * Camera mounting, to work out how high a CELL opening is. Measure on the robot: the lens height
     * above the TILES, and how far the camera is tilted up from level (the HIVE is high, so the
     * camera looks up). Same numbers as BioBuzzAutoAprilTag.
     */
    public static double CAMERA_HEIGHT_IN = 12.0;
    public static double CAMERA_PITCH_DEG = 30.0;
    /*
     * When only one CELL of a HIVE is in view, it counts as up only above this height. The up CELL's
     * opening is about 53.5-65.6 in. above the TILES. Check it on a real field: read the "height"
     * telemetry with the HIVE tipped each way, and set this halfway between the two readings.
     */
    public static double UP_CELL_MIN_HEIGHT_IN = 45.0;

    // Must match the webcam's name in the Driver Hub configuration.
    public static final String WEBCAM_NAME = "Webcam 1";

    // Limelight 3A (RobotSwitches.USE_LIMELIGHT): its configuration name, its AprilTag pipeline, and
    // how old (ms) a Limelight result may be before it counts as "nothing seen".
    public static final String LIMELIGHT_NAME = "limelight";
    public static int LIMELIGHT_PIPELINE = 0;
    public static double LIMELIGHT_MAX_STALENESS_MS = 100;

    private final boolean enabled = RobotSwitches.USE_CAMERA;
    private final boolean useLimelight = RobotSwitches.USE_CAMERA && RobotSwitches.USE_LIMELIGHT;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Limelight3A limelight;
    private final AprilTagLibrary library = AprilTagGameDatabase.getBioBuzzTagLibrary();
    // From the last Limelight result: each CELL's tag IDs ("34 35"), and tags that belong to no CELL.
    private final Map<String, String> limelightIds = new LinkedHashMap<>();
    private final StringBuilder limelightOtherTags = new StringBuilder();

    public AprilTagVision(HardwareMap hardwareMap) {
        if (!enabled) {
            return;
        }
        if (useLimelight) {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();   // no results at all until start() is called
            return;
        }
        // SDK 12 ships the BIOBUZZ tags as four clusters, with each tag's size and place in its cluster.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        // Show the camera picture on FTC Dashboard, at most 10 pictures per second.
        if (RobotSwitches.USE_DASHBOARD) {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /*
     * Returns the UP CELL the camera can see right now (the nearer one if it sees both HIVES' up
     * CELLS), or null if it sees no up CELL. Its ftcPose is the CELL opening: range in inches,
     * bearing and yaw in degrees.
     */
    public AprilTagClusterDetection closestTag() {
        List<AprilTagClusterDetection> cells = visibleCells();
        AprilTagClusterDetection best = null;
        for (AprilTagClusterDetection cell : cells) {
            if (isUp(cell, cells) && (best == null || cell.ftcPose.range < best.ftcPose.range)) {
                best = cell;
            }
        }
        return best;
    }

    /*
     * Height of a CELL opening above the TILES, in inches. ftcPose.z is "up" in the CAMERA's picture;
     * the camera is tilted up by CAMERA_PITCH_DEG, so we turn the camera's forward (y) and up (z)
     * into true up: y * sin(pitch) + z * cos(pitch), then add the lens height.
     */
    public double cellHeightInches(AprilTagDetection cell) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        return CAMERA_HEIGHT_IN + cell.ftcPose.y * Math.sin(pitch) + cell.ftcPose.z * Math.cos(pitch);
    }

    // True when this CELL is the up one of its HIVE (see the comment at the top of this file).
    private boolean isUp(AprilTagClusterDetection cell, List<AprilTagClusterDetection> cells) {
        double height = cellHeightInches(cell);
        for (AprilTagClusterDetection other : cells) {
            if (other != cell && hive(other).equals(hive(cell))) {
                return height > cellHeightInches(other);   // both CELLS of this HIVE in view
            }
        }
        return height > UP_CELL_MIN_HEIGHT_IN;             // only this CELL in view
    }

    // "RED" or "BLUE": which HIVE a CELL belongs to, from its cluster name (e.g. "RED SCORING").
    private static String hive(AprilTagClusterDetection cell) {
        return cell.metadata.shortName.startsWith("RED") ? "RED" : "BLUE";
    }

    // Every HIVE CELL (tag cluster) the camera sees right now; empty when the camera is off.
    private List<AprilTagClusterDetection> visibleCells() {
        List<AprilTagClusterDetection> cells = new ArrayList<>();
        if (!enabled) {
            return cells;
        }
        if (useLimelight) {
            return limelightCells();
        }
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) {
            return cells;
        }
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cell = (AprilTagClusterDetection) detection;
                if (cell.metadata != null && cell.ftcPose != null) {
                    cells.add(cell);
                }
            }
        }
        return cells;
    }

    // True when the target is straight ahead: less than AIM_TOLERANCE_DEG to either side.
    public boolean isAimedAt(AprilTagDetection tag) {
        return tag != null && tag.ftcPose != null && Math.abs(tag.ftcPose.bearing) < AIM_TOLERANCE_DEG;
    }

    /*
     * The "rotate" power for MecanumDriveSubsystem.drive() that points the robot at a target.
     * Bearing is positive when the target is to the LEFT, but in drive() a positive rotate turns
     * RIGHT, so we flip the sign. Within AIM_TOLERANCE_DEG we stop turning.
     */
    public double aimTurn(double bearingDegrees) {
        if (Math.abs(bearingDegrees) < AIM_TOLERANCE_DEG) {
            return 0;
        }
        return -Range.clip(AIM_TURN_GAIN * bearingDegrees, -AIM_MAX_TURN, AIM_MAX_TURN);
    }

    // One line per HIVE CELL the camera sees: name, UP or down, height, how many of its tags, distance, angle.
    public void addTelemetry(Telemetry telemetry) {
        if (!enabled) {
            return;
        }
        if (useLimelight) {
            telemetry.addData("Camera", "Limelight 3A, pipeline %d, %s", LIMELIGHT_PIPELINE,
                    limelight.isConnected() ? "connected" : "NOT CONNECTED (check USB and the configuration name)");
            telemetry.addData("Limelight tag size", "%.1f mm (set this in the pipeline)", limelightTagSizeMm());
        }
        List<AprilTagClusterDetection> cells = visibleCells();
        if (cells.isEmpty()) {
            telemetry.addData("HIVE tags", "none seen");
            return;
        }
        for (AprilTagClusterDetection cell : cells) {
            telemetry.addData(cell.metadata.shortName, "%s, height %.1f in, %d%% of tags, %.1f in away, %.1f deg %s",
                    isUp(cell, cells) ? "UP" : "down", cellHeightInches(cell), cell.percentClusterFound,
                    cell.ftcPose.range, Math.abs(cell.ftcPose.bearing), cell.ftcPose.bearing > 0 ? "left" : "right");
        }
    }

    // The tag IDs in each HIVE cluster (SDK 12's BIOBUZZ library).
    private static String clusterIds(String shortName) {
        switch (shortName) {
            case "RED SCORING": return "30-33";
            case "RED AUDIENCE": return "34-37";
            case "BLUE AUDIENCE": return "38-41";
            case "BLUE SCORING": return "42-45";
            default: return "?";
        }
    }

    /*
     * Everything the camera sees right now, in one line for logs (no commas, so it fits a CSV column),
     * or "none", or "camera off". SDK 12 reports a HIVE cluster's tags together, not one by one, so each
     * CELL shows its cluster's tag IDs and how many of its 4 tags were found:
     *   "RED AUDIENCE IDs 34-37 2/4 tags UP 35 in 2 deg left"
     * A tag that is not part of a HIVE is listed by its own ID ("tag 12").
     * details = false leaves out distance and angle, so it only changes when WHAT is seen changes.
     */
    public String seenSummary(boolean details) {
        if (!enabled) {
            return "camera off";
        }
        List<AprilTagClusterDetection> cells = visibleCells();
        StringBuilder sb = new StringBuilder();
        for (AprilTagClusterDetection cell : cells) {
            if (sb.length() > 0) {
                sb.append("; ");
            }
            // The Limelight reports each tag's real ID; the webcam only its cluster's ID range.
            String ids = useLimelight ? limelightIds.get(cell.metadata.shortName) : clusterIds(cell.metadata.shortName);
            sb.append(String.format(Locale.US, "%s IDs %s %d/4 tags %s", cell.metadata.shortName,
                    ids, Math.round(cell.percentClusterFound * 4 / 100.0),
                    isUp(cell, cells) ? "UP" : "down"));
            if (details) {
                sb.append(String.format(Locale.US, " %.0f in %.0f deg %s", cell.ftcPose.range,
                        Math.abs(cell.ftcPose.bearing), cell.ftcPose.bearing > 0 ? "left" : "right"));
            }
        }
        if (useLimelight) {
            if (limelightOtherTags.length() > 0) {
                sb.append(sb.length() > 0 ? "; " : "").append(limelightOtherTags);
            }
            return sb.length() == 0 ? "none" : sb.toString();
        }
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections != null) {
            for (AprilTagDetection d : detections) {
                if (d instanceof AprilTagSingleDetection) {
                    if (sb.length() > 0) {
                        sb.append("; ");
                    }
                    sb.append("tag ").append(((AprilTagSingleDetection) d).id);
                }
            }
        }
        return sb.length() == 0 ? "none" : sb.toString();
    }

    /*
     * LIMELIGHT: the CELLS in the Limelight's latest result. It reports one result per tag; the BIOBUZZ
     * library tells which CELL (cluster) each tag belongs to, and the CELL's pose is the average of its
     * tags' poses. Nothing is seen when the result is invalid or older than LIMELIGHT_MAX_STALENESS_MS.
     */
    private List<AprilTagClusterDetection> limelightCells() {
        List<AprilTagClusterDetection> cells = new ArrayList<>();
        limelightIds.clear();
        limelightOtherTags.setLength(0);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
            return cells;
        }
        Map<String, double[]> sums = new LinkedHashMap<>();   // per CELL: x, y, z, yaw, tag count
        Map<String, AprilTagClusterMetadata> clusters = new LinkedHashMap<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            AprilTagClusterMetadata cluster = library.lookupCluster(id);
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (cluster == null || pose == null) {
                limelightOtherTags.append(limelightOtherTags.length() > 0 ? "; " : "").append("tag ").append(id);
                continue;
            }
            // Limelight camera space: x right, y DOWN, z forward. ftcPose: x right, y forward, z UP.
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) {
                continue;   // no 3D pose: is "full 3D" on in the pipeline?
            }
            String name = cluster.shortName;
            double[] s = sums.get(name);
            if (s == null) {
                s = new double[5];
                sums.put(name, s);
                clusters.put(name, cluster);
                limelightIds.put(name, String.valueOf(id));
            } else {
                limelightIds.put(name, limelightIds.get(name) + " " + id);
            }
            s[0] += p.x;
            s[1] += p.z;
            s[2] += -p.y;
            s[3] += pose.getOrientation().getYaw(AngleUnit.DEGREES);
            s[4] += 1;
        }
        for (Map.Entry<String, double[]> e : sums.entrySet()) {
            double[] s = e.getValue();
            double n = s[4];
            double x = s[0] / n, y = s[1] / n, z = s[2] / n, yaw = s[3] / n;
            double range = Math.hypot(x, y);                         // flat, like the SDK's ftcPose.range
            double bearing = Math.toDegrees(Math.atan2(-x, y));      // + = left, like the SDK
            double elevation = Math.toDegrees(Math.atan2(z, range));
            AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(x, y, z, yaw, 0, 0, range, bearing, elevation);
            cells.add(new AprilTagClusterDetection((int) Math.round(100 * n / 4), clusters.get(e.getKey()),
                    DistanceUnit.INCH, ftcPose, null, null, System.nanoTime()));
        }
        return cells;
    }

    // The BIOBUZZ AprilTag size in mm, from the SDK library, for the Limelight pipeline setting.
    private double limelightTagSizeMm() {
        AprilTagMetadata tag = library.lookupTag(30);
        return tag == null ? 0 : tag.distanceUnit.toMm(tag.tagsize);
    }

    // Turn the camera off. Call this in stop() so the next OpMode can use it.
    public void close() {
        if (!enabled) {
            return;
        }
        if (useLimelight) {
            limelight.stop();
            return;
        }
        if (RobotSwitches.USE_DASHBOARD) {
            FtcDashboard.getInstance().stopCameraStream();
        }
        visionPortal.close();
    }
}
