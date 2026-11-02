package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.LaunchAssist;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/*
 * LAUNCH ASSIST TEST: shows how the camera (AprilTagVision) and LaunchAssist help the launcher,
 * without moving anything. No motor or servo runs: every line says what the TeleOp WOULD do.
 *
 *  1. CAMERA      which up CELL it sees: distance, angle, and the launcher's angle to it.
 *  2. AIM         the turn the driver's left bumper would give, and "aimed" when the launcher points at it.
 *  3. SPEED       the flywheel speed for that distance (LauncherSubsystem.speedForRange), next to the
 *                 fixed TARGET_VELOCITY, and whether the distance is in the shooting zone.
 *  4. POSE        LaunchAssist: the nearest saved good launch pose for this CELL, how far off the robot
 *                 is, and the forward / strafe / turn the align button would drive.
 *  5. VERDICT     READY when the camera sees an up CELL, the launcher is aimed, the distance is in the
 *                 zone, and the pose matches a saved one (or none is saved yet).
 *
 * Try it: point the camera at the HIVE and press A ("launch here"), then B ("that one scored"). That
 * saves a test launch pose. Move the robot (or the camera) and watch POSE tell you how to get back.
 * Test poses go to /sdcard/FIRST/BioBuzz/launch_log_test.csv, never into the TeleOp's launch_log.csv.
 *
 * Needs RobotSwitches.USE_CAMERA and USE_LAUNCH_ASSIST on. The drive and the launcher may be off.
 *
 * DASHBOARD
 *   Graph:          "cell range", "launcher bearing", "distance speed", "shot target",
 *                   "align forward", "align strafe", "align turn".
 *   Configuration:  LaunchAssist       TAG_RANGE_TOL_IN, TAG_BEARING_TOL_DEG, TAG_YAW_TOL_DEG (how close
 *                                      counts as OK), ALIGN_KP_*, ALIGN_MAX_POWER (how hard align drives)
 *                   AprilTagVision     AIM_TURN_GAIN, AIM_MAX_TURN, AIM_TOLERANCE_DEG, LAUNCHER_OFFSET_*
 *                   LauncherSubsystem  USE_DISTANCE_SPEED, NEAR/FAR_RANGE_IN, NEAR/FAR_VELOCITY,
 *                                      ZONE_MIN/MAX_RANGE_IN, TARGET_VELOCITY
 *   Changes work at once. They are lost when the robot restarts: copy the keepers into the .java files.
 */
@TeleOp(name = "Launch Assist Test", group = "StarterBot")
public class BioBuzzLaunchAssistTest extends OpMode {

    // Its own log, so test launches never become TeleOp launch poses. Built-in poses are still loaded.
    public static final String TEST_LOG_PATH = "BioBuzz/launch_log_test.csv";

    private MecanumDriveSubsystem drive = null;
    private LauncherSubsystem launcher = null;
    private AprilTagVision vision = null;
    private LaunchAssist launchAssist = null;

    @Override
    public void init() {
        // Test OpModes always send to FTC Dashboard: they never run in a match.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Only for the odometry pose in the log and the flywheel speed in a snapshot; neither moves.
        drive = new MecanumDriveSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        vision = new AprilTagVision(hardwareMap);
        launchAssist = new LaunchAssist(LaunchAssist.DEFAULTS_RESOURCE, TEST_LOG_PATH);

        telemetry.addData("Camera hardware", vision.hardwareStatus());
        telemetry.addData("Launch assist", launchAssist.isEnabled() ? "on" : "OFF: set RobotSwitches.USE_LAUNCH_ASSIST = true");
        telemetry.addLine("Point the camera at the HIVE. A = launch here, B = that one scored. Nothing moves.");
    }

    @Override
    public void init_loop() {
        test();
    }

    @Override
    public void loop() {
        test();
    }

    private void test() {
        drive.update();   // odometry only (it stays at 0 with the drive off)

        // ---------------- 1. CAMERA ----------------
        AprilTagClusterDetection cell = vision.aimedUpCell();   // the up CELL we point at, or null
        double launcherBearing = cell != null ? vision.launcherBearing(cell) : 0;
        if (cell == null) {
            telemetry.addData("1 CAMERA", "no up CELL in view: turn toward the HIVE");
        } else {
            telemetry.addData("1 CAMERA", "%s, %.1f in away, camera %.1f° %s, launcher %.1f° %s, yaw %.1f°",
                    cell.metadata.shortName, cell.ftcPose.range,
                    Math.abs(cell.ftcPose.bearing), cell.ftcPose.bearing > 0 ? "left" : "right",
                    Math.abs(launcherBearing), launcherBearing > 0 ? "left" : "right", cell.ftcPose.yaw);
        }

        // ---------------- 2. AIM (driver left bumper) ----------------
        boolean aimed = cell != null && vision.isAimedAt(cell);
        if (cell == null) {
            telemetry.addData("2 AIM", "nothing to aim at");
        } else if (aimed) {
            telemetry.addData("2 AIM", "AIMED (within %.0f°)", AprilTagVision.AIM_TOLERANCE_DEG);
        } else {
            telemetry.addData("2 AIM", "left bumper would turn %+.2f power", vision.aimTurn(launcherBearing));
        }

        // ---------------- 3. SPEED (the camera's distance sets the flywheel speed) ----------------
        launcher.setCell(cell);   // keeps the last distance when the CELL is out of view
        double range = launcher.getCellRange();
        boolean inZone = launcher.isInShootingZone();
        if (Double.isNaN(range)) {
            telemetry.addData("3 SPEED", "no CELL seen yet: fixed %.0f ticks/s", LauncherSubsystem.TARGET_VELOCITY);
        } else {
            telemetry.addData("3 SPEED", "%.0f ticks/s for %.0f in (fixed %.0f), using %s, %s",
                    LauncherSubsystem.speedForRange(range), range, LauncherSubsystem.TARGET_VELOCITY,
                    LauncherSubsystem.USE_DISTANCE_SPEED ? "DISTANCE" : "FIXED (USE_DISTANCE_SPEED = false)",
                    inZone ? "IN zone" : String.format("OUTSIDE the %.0f-%.0f in zone",
                            LauncherSubsystem.ZONE_MIN_RANGE_IN, LauncherSubsystem.ZONE_MAX_RANGE_IN));
        }
        telemetry.addData("   shot target", "%.0f ticks/s", launcher.getShootTarget());

        // ---------------- 4. POSE (LaunchAssist) ----------------
        LaunchAssist.Status pose = launchAssist.update(drive, cell, launcher);
        if (gamepad1.aWasPressed()) {
            launchAssist.recordLaunch();             // "launch here": one LAUNCH line in the test log
        }
        if (gamepad1.bWasPressed()) {
            launchAssist.saveLastLaunch(gamepad1);   // "that one scored": a test launch pose
        }
        telemetry.addLine("4 POSE (LaunchAssist)");
        launchAssist.addDriverLines(telemetry, false);   // "Launch pose", saved count, "Saved ..." message
        if (pose.hasTarget) {
            telemetry.addData("   align would drive", "forward %+.2f, strafe %+.2f, turn %+.2f",
                    pose.forward, pose.strafe, pose.rotate);
        }

        // ---------------- 5. VERDICT ----------------
        boolean poseOk = pose.ok || (cell != null && !pose.hasTarget);   // no saved pose yet: don't block
        String verdict;
        if (cell == null) {
            verdict = "NOT READY: no up CELL in view";
        } else if (!aimed) {
            verdict = "NOT READY: aim first (left bumper)";
        } else if (!inZone) {
            verdict = "NOT READY: distance outside the shooting zone";
        } else if (!poseOk) {
            verdict = "NOT READY: not at a saved launch pose (align button)";
        } else {
            verdict = "READY TO LAUNCH";
        }
        telemetry.addData("5 VERDICT", verdict);

        // ---------------- Dashboard graph lines (0 when there is nothing to show) ----------------
        telemetry.addData("cell range", cell != null ? cell.ftcPose.range : 0);
        telemetry.addData("launcher bearing", launcherBearing);
        telemetry.addData("distance speed", Double.isNaN(range) ? 0 : LauncherSubsystem.speedForRange(range));
        telemetry.addData("shot target", launcher.getShootTarget());
        telemetry.addData("align forward", pose.forward);
        telemetry.addData("align strafe", pose.strafe);
        telemetry.addData("align turn", pose.rotate);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }
}
