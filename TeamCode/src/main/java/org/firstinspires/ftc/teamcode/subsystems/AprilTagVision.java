package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

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
 *  - aimTurn(bearing): the turn power that points the robot at it; pass launcherBearing(cell) so
 *    the launcher, not the camera, ends up pointing at the opening (LAUNCHER_OFFSET_* below)
 *  - streams the camera picture to FTC Dashboard (http://192.168.43.1:8080/dash)
 *
 * Making the Logitech C270 webcam reliable (details at "Webcam picture settings" below):
 *  - 640x480 pictures, the only size the SDK has a lens calibration for;
 *  - DECIMATION 2 instead of the SDK's 3, so the tags are found farther away;
 *  - a short manual exposure (EXPOSURE_MS, GAIN), so the picture doesn't smear while turning;
 *  - CELLS from a picture older than MAX_RESULT_AGE_MS count as not seen (a stalled camera);
 *  - each CELL's range is averaged over RANGE_AVERAGE_FRAMES pictures, for a steadier flywheel speed.
 * The "Camera" telemetry line shows what the camera actually got.
 *
 * Which CELL is up: each HIVE is a see-saw with two CELLS (red HIVE: RED SCORING + RED AUDIENCE,
 * blue HIVE: BLUE SCORING + BLUE AUDIENCE), and the up CELL's opening is the higher one.
 *  - Both CELLS of a HIVE in view: the higher one is up.
 *  - Only one in view: it is up if it is higher than UP_CELL_MIN_HEIGHT_IN.
 * If up CELLS of both HIVES are in view, the nearer one wins. If only a down CELL is in view,
 * closestTag() returns null, so we never aim at the down CELL.
 *
 * Bearing is in degrees. Positive bearing means the CELL opening is to the robot's LEFT.
 *
 * ALLIANCE FILTER (optional): setAlliance(RED or BLUE) makes closestTag() and aimedUpCell() return
 * only that alliance's HIVE, so aiming, the flywheel speed and LaunchAssist never pick the other HIVE.
 * Until it is set (and in the Auto, which never sets it) any HIVE counts. The TeleOp uses
 * aimedUpCell(): without an alliance it takes the up CELL the robot points at, so it follows the
 * driver's aim. upCellSummary() tells the drive team which CELL of their HIVE is tilted up.
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
    // Planned Logitech C270 mount (Camera Mount C270 Clip Front - Launcher Channel, beside the +X launcher
    // channel, looking FORWARD like the v6 shot). Lens 12.6 in above the TILES. Re-measure on the built robot.
    // 38 degrees up (was 45): the CELL's tag row is about 49.7 in above the TILES, 37.1 in above the lens.
    // From the closest launch spot (NEAR_RANGE_IN, tags ~30 in away along the floor) the camera looks up
    // atan(37.1 / 30) = 51 degrees, from the farthest (FAR_RANGE_IN, ~79 in) atan(37.1 / 79) = 25 degrees.
    // Halfway, (51 + 25) / 2 = 38, keeps both inside the C270's 32.5 degree tall picture with ~3 degrees to
    // spare at each edge; at 45 the picture's bottom edge is 29 degrees, so the far spot was out of view.
    // Any tilt from about 36 to 40 works (docs/vision-deck, slide 9). The mount must really be tilted 38:
    // measure it, and re-save the launch poses after re-tilting.
    public static double CAMERA_HEIGHT_IN = 12.6;
    public static double CAMERA_PITCH_DEG = 38.0;

    /*
     * Where the launcher's line of fire is, seen from the camera lens, so aiming points the LAUNCHER
     * (not the camera) at the CELL opening. Measure on the robot, looking the way the ball flies:
     * LAUNCHER_OFFSET_RIGHT_IN = how far the launcher wheel's center line is to the RIGHT of the lens
     * (negative = left), LAUNCHER_OFFSET_FORWARD_IN = how far the ball's release point is in front of
     * the lens (negative = behind). 0, 0 = the camera sits on the launcher's line of fire.
     * The camera must look the same way the ball flies (in CAD the Julia Runway v6 throws toward the
     * robot's FRONT, the intake side): if it looks the other way, aiming turns the launcher away from the HIVE.
     */
    // Front C270 mount (Camera Mount C270 Clip Front): looking forward, the launcher is 140 mm to the
    // RIGHT of the lens and the v6 release point 52 mm (2.0 in) BEHIND it.
    // (Back-launch Runway v5 with the rear clip mount was -5.5 / 0.4.)
    public static double LAUNCHER_OFFSET_RIGHT_IN = 5.5;
    public static double LAUNCHER_OFFSET_FORWARD_IN = -2.0;
    /*
     * When only one CELL of a HIVE is in view, it counts as up only above this height. The up CELL's
     * opening is about 53.5-65.6 in. above the TILES. Check it on a real field: read the "height"
     * telemetry with the HIVE tipped each way, and set this halfway between the two readings.
     */
    public static double UP_CELL_MIN_HEIGHT_IN = 45.0;
    // aimedUpCell(): two up CELLS less than this many degrees apart count as the same direction.
    public static double SAME_DIRECTION_DEG = 5.0;

    /*
     * ---- Webcam picture settings (Logitech C270; not used with the Limelight) ----
     * Background: docs/vision-deck (slides "C270 camera" and "Better software").
     *
     * PICTURE SIZE is fixed at 640x480 (CAMERA_RESOLUTION below). The SDK has a built-in lens
     * calibration for the C270 at that size ONLY. Range and height come from the calibration, so at
     * any other size (e.g. 1280x720) every distance would be wrong. Don't change it.
     *
     * DECIMATION: the tag finder first looks for squares in a smaller copy of the picture, DECIMATION
     * times smaller each way. Bigger = faster but sees less far. The SDK default is 3, which finds our
     * 3.25 in tags to only about 6.5 ft, right at the far edge of what our 45-degree camera can see.
     * 2 reaches about 9.75 ft at about 22 pictures a second (SDK sample ConceptAprilTag, 2 in tag
     * distances scaled by 3.25 / 2). Check "Camera ... fps" on the Driver Hub after changing it.
     *
     * EXPOSURE: with automatic exposure the C270 keeps its shutter open a long time under venue
     * lights, so the picture smears while the robot turns and the tags are lost just when we aim.
     * MANUAL_EXPOSURE = true uses a short EXPOSURE_MS and makes up the brightness with GAIN.
     * ⚠️ 6 ms / gain 250 are starting points, not measured on our camera. Find ours with the SDK sample
     * "Optimize AprilTag Exposure" (copy ConceptAprilTagOptimizeExposure into TeamCode, remove
     * @Disabled): the SHORTEST exposure that still finds the tags, with the gain high. Both are clipped
     * to what the camera allows. MANUAL_EXPOSURE = false gives the camera back its automatic exposure.
     * All four values can be changed live in FTC Dashboard; applyCameraSettings() sends a change to
     * the camera within a loop or two (same idea as the subsystems' sendPidfIfChanged()).
     */
    public static int DECIMATION = 2;
    public static boolean MANUAL_EXPOSURE = true;
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 250;

    /*
     * OLD PICTURES: getDetections() hands back the result of the last picture it finished. If the
     * camera stalls (loose USB cable, static), that result never changes, and the robot would keep
     * aiming at an angle that is no longer true. A CELL seen in a picture older than
     * MAX_RESULT_AGE_MS counts as not seen. At 22 pictures a second a new one comes every ~45 ms and
     * the tag finder needs about as long again, so 200 ms means "several pictures in a row missing".
     * 0 = never drop (the old behaviour). The Limelight has its own: LIMELIGHT_MAX_STALENESS_MS.
     */
    public static double MAX_RESULT_AGE_MS = 200;

    /*
     * STEADIER RANGE: the bearing from a tag is very exact, but its range wobbles by an inch or two,
     * more the farther away it is, and the flywheel speed is picked from the range. So each CELL's
     * ftcPose.range is the average over its last RANGE_AVERAGE_FRAMES pictures (1 = no averaging).
     * Only the range is averaged: bearing, x, y, z and yaw stay as measured, so aiming stays quick.
     * The average starts over when the CELL was out of view for RANGE_RESET_MS, or when a new range
     * is more than RANGE_JUMP_IN away from the average (the HIVE tipped, or we drove fast), because
     * averaging across a jump would give a distance that was never true.
     */
    public static int RANGE_AVERAGE_FRAMES = 4;
    public static double RANGE_RESET_MS = 300;
    public static double RANGE_JUMP_IN = 6.0;

    // Must match the webcam's name in the Driver Hub configuration.
    public static final String WEBCAM_NAME = "Webcam 1";
    /*
     * Our own lens calibration (fx, fy, cx, cy in pixels), for CAMERA_RESOLUTION. false = the SDK's
     * built-in C270 calibration for 640x480 (fx = fy = 822.3, cx = 319.5, cy = 242.5).
     * Off: the values below were measured at a different resolution (cx/cy should be near 320/240 at
     * 640x480), so with them every range read about half the real distance (433 / 822 = 0.53).
     * Recalibrate at 640x480 before turning it back on. Can be changed in Dashboard; read at INIT.
     */
    public static boolean USE_LENS_INTRINSICS = false;
    public static double LENS_FX = 433.137, LENS_FY = 433.137, LENS_CX = 422.08, LENS_CY = 354.865;
    // The only size with a built-in C270 calibration (see "Webcam picture settings" above).
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);

    // Limelight 3A (RobotSwitches.USE_LIMELIGHT): its configuration name, its AprilTag pipeline, and
    // how old (ms) a Limelight result may be before it counts as "nothing seen".
    public static final String LIMELIGHT_NAME = "limelight";
    public static int LIMELIGHT_PIPELINE = 0;
    public static double LIMELIGHT_MAX_STALENESS_MS = 100;

    // Which alliance's HIVE closestTag() may return; null = any (see ALLIANCE FILTER at the top).
    public enum Alliance { RED, BLUE }
    private Alliance alliance = null;

    private final boolean enabled;
    // Was the camera found in the hardware configuration? Shown at INIT and written to the hub's log.
    private String hardwareStatus = "off (RobotSwitches.USE_CAMERA = false)";
    private final boolean useLimelight;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;   // null in plain camera mode
    // Plain camera mode (Camera Test): picture only, no AprilTag processor, no exposure settings.
    private final boolean plain;
    private Limelight3A limelight;
    private final AprilTagLibrary library = AprilTagGameDatabase.getBioBuzzTagLibrary();
    // From the last Limelight result: each CELL's tag IDs ("34 35"), and tags that belong to no CELL.
    private final Map<String, String> limelightIds = new LinkedHashMap<>();
    private final StringBuilder limelightOtherTags = new StringBuilder();

    // What applyCameraSettings() last sent to the camera, so it only sends a CHANGE.
    private int sentDecimation = -1;
    private Boolean sentManualExposure = null;   // null = nothing sent yet
    private int sentExposureMs = -1;
    private int sentGain = -1;
    private long manualModeSetAtNs = 0;          // when we switched the camera to manual exposure
    private String cameraSettingsText = "waiting for the camera";

    // Range averaging (RANGE_AVERAGE_FRAMES), one history per CELL name.
    private static class RangeHistory {
        final ArrayDeque<Double> ranges = new ArrayDeque<>();
        long lastFrameNs;   // frameAcquisitionNanoTime of the newest range in the history
    }
    private final Map<String, RangeHistory> rangeHistories = new HashMap<>();

    public AprilTagVision(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /*
     * useCamera = false switches the camera off for one OpMode only (e.g. a launcher test): no camera
     * is looked for, and every method acts as if nothing is in view. RobotSwitches.USE_CAMERA = false
     * still switches it off for every OpMode.
     */
    public AprilTagVision(HardwareMap hardwareMap, boolean useCamera) {
        this(hardwareMap, useCamera, false);
    }

    /*
     * plainCamera = true: only the webcam picture (live view on the HDMI monitor and FTC Dashboard), like
     * the SDK's UtilityCameraFrameCapture: no AprilTag processor and no exposure / gain settings, so
     * nothing is ever seen. For Camera Test, to tell a camera problem from a processor or settings problem.
     */
    public AprilTagVision(HardwareMap hardwareMap, boolean useCamera, boolean plainCamera) {
        plain = plainCamera;
        boolean on = RobotSwitches.USE_CAMERA && useCamera;
        WebcamName webcam = null;
        if (on && RobotSwitches.USE_LIMELIGHT) {
            hardwareStatus = "Limelight 3A \"" + LIMELIGHT_NAME + "\"";
        } else if (on) {
            // tryGet() gives null instead of crashing, so a missing webcam is reported, not an error.
            webcam = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
            if (webcam == null) {
                hardwareStatus = "NOT FOUND: no webcam named \"" + WEBCAM_NAME + "\" in the active configuration";
                on = false;
            } else if (!webcam.isAttached()) {
                hardwareStatus = "\"" + WEBCAM_NAME + "\" is in the configuration but NOT PLUGGED IN (blue USB 3.0 port)";
                on = false;
            } else {
                hardwareStatus = "found \"" + WEBCAM_NAME + "\", plugged in";
            }
        } else if (RobotSwitches.USE_CAMERA) {
            hardwareStatus = "off for this OpMode";
        }
        RobotLog.ii("AprilTagVision", "camera hardware: %s", hardwareStatus);
        enabled = on;
        useLimelight = enabled && RobotSwitches.USE_LIMELIGHT;
        if (!enabled) {
            return;
        }
        if (useLimelight) {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();   // no results at all until start() is called
            return;
        }
        if (plain) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(CAMERA_RESOLUTION)
                    .build();
            cameraSettingsText = "PLAIN camera: no AprilTag processor, no exposure settings";
            if (RobotSwitches.USE_DASHBOARD) {
                FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
            }
            return;
        }
        // SDK 12 ships the BIOBUZZ tags as four clusters, with each tag's size and place in its cluster.
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
        if (USE_LENS_INTRINSICS) {
            builder.setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY);
        }
        aprilTag = builder.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(CAMERA_RESOLUTION)   // 640x480: the C270's calibrated size
                .addProcessor(aprilTag)
                .build();
        // Decimation can be set right away. Exposure and gain must wait until the camera is
        // streaming, which takes a second or two, so applyCameraSettings() finishes them later.
        applyCameraSettings();

        // Show the camera picture on FTC Dashboard, at most 10 pictures per second.
        if (RobotSwitches.USE_DASHBOARD) {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
        }
    }

    // "found "Webcam 1", plugged in", "NOT FOUND: ..." and so on, from INIT.
    public String hardwareStatus() {
        return hardwareStatus;
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
            if (alliance != null && !hive(cell).equals(alliance.name())) {
                continue;   // the other alliance's HIVE
            }
            if (isUp(cell, cells) && (best == null || cell.ftcPose.range < best.ftcPose.range)) {
                best = cell;
            }
        }
        return best;
    }

    /*
     * For the TeleOp: the up CELL the robot is POINTED at (smallest bearing), so the target is the one
     * the driver aims at, not just the nearest. Only our HIVE when setAlliance() was used. Two CELLS
     * within SAME_DIRECTION_DEG of each other are in about the same direction (from one side the other
     * HIVE's CELL can sit right behind ours): then the nearer one wins.
     */
    public AprilTagClusterDetection aimedUpCell() {
        List<AprilTagClusterDetection> cells = visibleCells();
        AprilTagClusterDetection best = null;
        for (AprilTagClusterDetection cell : cells) {
            if (alliance != null && !hive(cell).equals(alliance.name())) {
                continue;   // the other alliance's HIVE
            }
            if (!isUp(cell, cells)) {
                continue;
            }
            if (best == null) {
                best = cell;
                continue;
            }
            double off = Math.abs(cell.ftcPose.bearing);
            double bestOff = Math.abs(best.ftcPose.bearing);
            boolean sameDirection = Math.abs(off - bestOff) <= SAME_DIRECTION_DEG;
            if (sameDirection ? cell.ftcPose.range < best.ftcPose.range : off < bestOff) {
                best = cell;
            }
        }
        return best;
    }

    // Only this alliance's HIVE counts from now on; null = any HIVE again.
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /*
     * Which CELL of our HIVE is tilted up, for the drive team, e.g. "RED AUDIENCE, your RIGHT
     * (audience end)". Left and right are as seen from behind our ALLIANCE wall. Taken from an up CELL
     * of our HIVE in view, or, when only our down CELL is in view, the other CELL of the HIVE.
     */
    public String upCellSummary() {
        if (!enabled) {
            return "camera off";
        }
        if (alliance == null) {
            return "alliance not set";
        }
        List<AprilTagClusterDetection> cells = visibleCells();
        String up = null;
        boolean fromDownCell = false;
        for (AprilTagClusterDetection cell : cells) {
            if (!hive(cell).equals(alliance.name())) {
                continue;
            }
            String name = cell.metadata.shortName;
            if (isUp(cell, cells)) {
                up = name;
                fromDownCell = false;
                break;
            }
            up = otherCell(name);
            fromDownCell = true;
        }
        if (up == null) {
            return "our HIVE not in view";
        }
        return String.format(Locale.US, "%s, your %s (%s end)%s", up, driverSide(up),
                up.contains("SCORING") ? "far" : "audience", fromDownCell ? ", seen from the down CELL" : "");
    }

    // The other CELL of the same HIVE: RED SCORING <-> RED AUDIENCE, BLUE SCORING <-> BLUE AUDIENCE.
    private static String otherCell(String name) {
        return name.contains("SCORING") ? name.replace("SCORING", "AUDIENCE") : name.replace("AUDIENCE", "SCORING");
    }

    /*
     * LEFT or RIGHT for a driver behind that alliance's wall. SCORING is the far-end CELL (BioBuzzField):
     * red drivers see the far end on their left; the field is turned 180 degrees for blue, so blue
     * drivers see it on their right.
     */
    private static String driverSide(String name) {
        boolean farEnd = name.contains("SCORING");
        boolean red = name.startsWith("RED");
        return farEnd == red ? "LEFT" : "RIGHT";
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

    /*
     * Intake (the robot's front) to the HIVE stand, from the camera: range minus STAND_OFFSET_IN. The
     * offset is the lens behind the intake plus the CELL opening set back inside the HIVE (CAD and
     * simulation: range 54.2 in with the front 15 in from the stand). Only right when the robot faces
     * the CELL square. Check once: tape-measure intake to stand, then STAND_OFFSET_IN = range - tape.
     */
    public static double STAND_OFFSET_IN = 39.2;

    public double intakeToStandInches(AprilTagDetection cell) {
        return cell.ftcPose.range - STAND_OFFSET_IN;
    }

    // One Driver Hub line with what the camera reads for this CELL (null = none in view).
    public void addReadingLine(Telemetry telemetry, AprilTagClusterDetection cell) {
        if (!enabled) {
            telemetry.addData("Camera reading", "camera off");
        } else if (cell == null) {
            telemetry.addData("Camera reading", "no up CELL in view");
        } else {
            telemetry.addData("Camera reading", "%s: range %.1f in, bearing %.1f°, yaw %.1f°, intake to stand ~%.0f in",
                    cell.metadata.shortName, cell.ftcPose.range, cell.ftcPose.bearing, cell.ftcPose.yaw,
                    intakeToStandInches(cell));
        }
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
        if (plain) {
            return cells;   // no AprilTag processor
        }
        // Every OpMode asks for CELLS every loop (also in init_loop), so this is where a camera
        // setting changed in FTC Dashboard, or still waiting for the camera to start, gets sent.
        applyCameraSettings();
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) {
            return cells;
        }
        long now = System.nanoTime();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cell = (AprilTagClusterDetection) detection;
                if (cell.metadata != null && cell.ftcPose != null && !isTooOld(cell, now)) {
                    cells.add(withAveragedRange(cell, now));
                }
            }
        }
        return cells;
    }

    /*
     * OLD PICTURES (MAX_RESULT_AGE_MS): frameAcquisitionNanoTime is when the camera took the picture,
     * on the same System.nanoTime() clock as `now`, so now - frame time is the result's age.
     */
    private static boolean isTooOld(AprilTagDetection cell, long now) {
        if (MAX_RESULT_AGE_MS <= 0) {
            return false;
        }
        double ageMs = (now - cell.frameAcquisitionNanoTime) / 1e6;
        return ageMs > MAX_RESULT_AGE_MS;
    }

    /*
     * STEADIER RANGE (RANGE_AVERAGE_FRAMES): the same CELL, with ftcPose.range replaced by the average
     * of its last few pictures. visibleCells() runs several times per loop and the camera is slower
     * than the loop, so a range is added only for a NEW picture (a new frameAcquisitionNanoTime);
     * otherwise the same picture would be counted many times. The SDK's pose is read-only, so we make
     * a new pose and a new CELL result with everything else copied unchanged.
     */
    private AprilTagClusterDetection withAveragedRange(AprilTagClusterDetection cell, long now) {
        String name = cell.metadata.shortName;
        RangeHistory history = rangeHistories.get(name);
        if (history == null) {
            history = new RangeHistory();
            rangeHistories.put(name, history);
        }
        double raw = cell.ftcPose.range;
        if (cell.frameAcquisitionNanoTime != history.lastFrameNs) {
            // Start over after a gap, or when the range jumped (tip, fast drive): see RANGE_RESET_MS.
            boolean gap = (cell.frameAcquisitionNanoTime - history.lastFrameNs) / 1e6 > RANGE_RESET_MS;
            boolean jump = !history.ranges.isEmpty() && Math.abs(raw - average(history.ranges)) > RANGE_JUMP_IN;
            if (gap || jump) {
                history.ranges.clear();
            }
            history.ranges.addLast(raw);
            while (history.ranges.size() > Math.max(1, RANGE_AVERAGE_FRAMES)) {
                history.ranges.removeFirst();   // keep only the newest RANGE_AVERAGE_FRAMES
            }
            history.lastFrameNs = cell.frameAcquisitionNanoTime;
        }
        double range = average(history.ranges);
        if (range == raw) {
            return cell;   // one picture so far (or RANGE_AVERAGE_FRAMES = 1): nothing to change
        }
        AprilTagPoseFtc p = cell.ftcPose;
        AprilTagPoseFtc pose = new AprilTagPoseFtc(p.x, p.y, p.z, p.yaw, p.pitch, p.roll, range, p.bearing, p.elevation);
        return new AprilTagClusterDetection(cell.percentClusterFound, cell.metadata, cell.distanceUnit,
                pose, cell.rawPose, cell.robotPose, cell.frameAcquisitionNanoTime);
    }

    private static double average(ArrayDeque<Double> values) {
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        return values.isEmpty() ? 0 : sum / values.size();
    }

    /*
     * Sends DECIMATION, MANUAL_EXPOSURE, EXPOSURE_MS and GAIN to the webcam, but only when one of them
     * changed since the last time (like the subsystems' sendPidfIfChanged()). It never waits: our
     * OpModes are iterative, and sleep() in a loop would freeze the robot. Instead each step that
     * isn't possible yet returns, and the next loop tries again:
     *  1. Decimation goes to the tag finder, which works from the start.
     *  2. Exposure and gain are camera (USB) controls, which only work once the camera is STREAMING,
     *     a second or two after INIT.
     *  3. The camera needs a moment after switching to manual exposure before it takes an exposure
     *     time (the SDK sample sleeps 50 ms there), so we switch first and send the time 50 ms later.
     */
    private void applyCameraSettings() {
        if (DECIMATION != sentDecimation) {
            aprilTag.setDecimation(Math.max(1, DECIMATION));
            sentDecimation = DECIMATION;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;   // step 2: try again next loop
        }
        boolean unchanged = sentManualExposure != null && sentManualExposure == MANUAL_EXPOSURE
                && (!MANUAL_EXPOSURE || (sentExposureMs == EXPOSURE_MS && sentGain == GAIN));
        if (unchanged) {
            return;
        }
        // GUARD: a webcam that refuses a setting must not stop the OpMode (and close the camera).
        // The picture then keeps the camera's own exposure, and the Camera line says what happened.
        try {
            sendExposureAndGain();
        } catch (RuntimeException e) {
            cameraSettingsText = "camera REFUSED the exposure/gain settings (" + e.getMessage() + "): using its own";
            sentManualExposure = MANUAL_EXPOSURE;   // don't try again every loop
            sentExposureMs = EXPOSURE_MS;
            sentGain = GAIN;
            RobotLog.ww("AprilTagVision", "camera settings refused: %s", e);
        }
    }

    // The exposure / gain part of applyCameraSettings(); it may throw if the webcam refuses a control.
    private void sendExposureAndGain() {
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        if (exposure == null) {
            cameraSettingsText = "this camera has no exposure control";
            sentManualExposure = MANUAL_EXPOSURE;   // nothing to send; don't ask again every loop
            return;
        }
        if (!MANUAL_EXPOSURE) {
            // Back to automatic: the Logitech webcams call their automatic mode "AperturePriority".
            exposure.setMode(exposure.isModeSupported(ExposureControl.Mode.AperturePriority)
                    ? ExposureControl.Mode.AperturePriority : ExposureControl.Mode.Auto);
            sentManualExposure = false;
            cameraSettingsText = "automatic exposure";
            return;
        }
        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);   // step 3: the time goes in 50 ms from now
            manualModeSetAtNs = System.nanoTime();
            return;
        }
        if ((System.nanoTime() - manualModeSetAtNs) / 1e6 < 50) {
            return;
        }
        // Clip to what this camera allows, so a typo in Dashboard can't send something it refuses.
        long ms = Math.max(exposure.getMinExposure(TimeUnit.MILLISECONDS),
                Math.min(exposure.getMaxExposure(TimeUnit.MILLISECONDS), EXPOSURE_MS));
        exposure.setExposure(ms, TimeUnit.MILLISECONDS);
        int g = GAIN;
        if (gain != null) {
            g = Range.clip(GAIN, gain.getMinGain(), gain.getMaxGain());
            gain.setGain(g);
        }
        sentManualExposure = true;
        sentExposureMs = EXPOSURE_MS;
        sentGain = GAIN;
        cameraSettingsText = String.format(Locale.US, "manual exposure %d ms, gain %d", ms, g);
    }

    // True when the launcher points at the target: less than AIM_TOLERANCE_DEG to either side.
    public boolean isAimedAt(AprilTagDetection tag) {
        return tag != null && tag.ftcPose != null && Math.abs(launcherBearing(tag)) < AIM_TOLERANCE_DEG;
    }

    /*
     * The bearing from the LAUNCHER to the target, in degrees (positive = left, like ftcPose.bearing).
     * It is the camera's bearing moved by LAUNCHER_OFFSET_RIGHT_IN / LAUNCHER_OFFSET_FORWARD_IN; with
     * both 0 it is exactly ftcPose.bearing. ftcPose.x is to the right of the lens, ftcPose.y forward.
     */
    public double launcherBearing(AprilTagDetection tag) {
        double right = tag.ftcPose.x - LAUNCHER_OFFSET_RIGHT_IN;
        double forward = tag.ftcPose.y - LAUNCHER_OFFSET_FORWARD_IN;
        return Math.toDegrees(Math.atan2(-right, forward));
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
        telemetry.addData("Camera hardware", hardwareStatus);
        if (!enabled) {
            return;
        }
        if (useLimelight) {
            telemetry.addData("Camera", "Limelight 3A, pipeline %d, %s", LIMELIGHT_PIPELINE,
                    limelight.isConnected() ? "connected" : "NOT CONNECTED (check USB and the configuration name)");
            telemetry.addData("Limelight tag size", "%.1f mm (set this in the pipeline)", limelightTagSizeMm());
        }
        List<AprilTagClusterDetection> cells = visibleCells();   // also sends camera settings (webcam)
        if (!useLimelight) {
            // Check here that the settings arrived: fps drops if DECIMATION is too low for the hub.
            telemetry.addData("Camera", "%s, %dx%d, %.0f fps, decimation %d, %s", visionPortal.getCameraState(),
                    CAMERA_RESOLUTION.getWidth(), CAMERA_RESOLUTION.getHeight(), visionPortal.getFps(),
                    sentDecimation, cameraSettingsText);
        }
        if (plain) {
            telemetry.addData("HIVE tags", "not looked for (PLAIN camera mode)");
            return;
        }
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
        List<AprilTagDetection> detections = plain ? null : aprilTag.getDetections();
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
