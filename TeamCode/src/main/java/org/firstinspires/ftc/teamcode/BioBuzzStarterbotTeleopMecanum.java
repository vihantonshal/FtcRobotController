/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.CellSightRumble;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchAssist;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels (goBILDA GripForce 104 mm) for the 2026-2027 FIRST® Tech Challenge.
 *
 * Each part of the robot is its own class in the "subsystems" folder. Each one knows HOW to do its
 * job and keeps its own tuning numbers:
 *   MecanumDriveSubsystem  drive motors + IMU + odometry, and how the sticks feel   (drive team)
 *   IntakeSubsystem        intake roller + servos                                  (intake team)
 *   LauncherSubsystem      launcher + windmill, on/off, ready, distance speed       (launcher team)
 *   AprilTagVision         webcam + HIVE tags                                      (vision team)
 *   LaunchAssist           log of launches, saved good launch poses, auto-align, LED (launch team)
 *   CellSightRumble        rumbles when an up CELL comes into view or is lost
 *   RobotSwitches          turn a part on or off while it is off the robot
 * This TeleOp only reads the gamepads and tells each part WHAT to do. The Auto uses the same classes.
 *
 * Controls. DRIVER = gamepad1. OPERATOR = gamepad2, or gamepad1 too when TWO_DRIVERS is false
 * (the buttons are chosen so nothing overlaps on one gamepad).
 *  AT INIT   (optional) DRIVER B: we are RED   X: we are BLUE. Aiming, the shot speed and launch
 *            assist then only use our alliance's HIVE, and the Driver Hub shows which CELL of our HIVE
 *            is tilted up ("HIVE up"). Not picked: the target is the up CELL the robot points at.
 *  DRIVER    left stick: drive and strafe         right stick X: turn
 *            left bumper (hold): turn toward the up CELL opening (AprilTag cluster)
 *            right bumper (hold): drive to the nearest saved good launch pose
 *                                 (D-pad up instead when TWO_DRIVERS is false)
 *            Driving is robot-centric: stick up = the way the robot's front faces.
 *            X: slow mode on/off (for lining up)
 *            Y: field-centric on/off, ONLY when RobotSwitches.USE_FIELD_CENTRIC is true (it is
 *               false now, so Y does nothing)
 *            back/share: field-centric only, reset "forward" to the way the robot faces now
 *  OPERATOR  right trigger: intake in             left trigger: intake out
 *            right bumper: launcher on/off (it keeps spinning; the gamepad rumbles when ready)
 *            A (hold): feed balls; the windmill only feeds once the launcher is at speed, in
 *                      pulses so the flywheel recovers between balls
 *            D-pad right/left: launcher a bit faster/slower (practice trim; the speed also follows
 *                              the distance to the up CELL, see LauncherSubsystem)
 *            B: the last launch scored, save its pose as a good launch pose
 *
 * The Driver Hub ALWAYS shows the launch lines: is the pose OK to launch, flywheel speed and
 * direction, and how many good launch poses are saved. The driver's gamepad LED is green when the
 * pose is OK, red when not, purple when an up CELL is in view but no good launch pose is saved for
 * it yet, blue when no up CELL is in view (PS4/PS5 gamepads). The driver's gamepad also blips twice
 * when an up CELL comes into view and rumbles once when it is lost (CellSightRumble).
 * While RobotSwitches.USE_DASHBOARD is true, the rest of the telemetry also goes to the Driver Station
 * and to FTC Dashboard (http://192.168.43.1:8080/dash), where the numbers can be graphed.
 * Every launch and every saved success is written to /sdcard/FIRST/BioBuzz/launch_log.csv.
 * The driver-feel numbers (DEADBAND, SLOW_MODE_SCALE, MAX_ACCEL_PER_SEC) are in MecanumDriveSubsystem.
 */

@Config
@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    // true: gamepad2 runs the intake and launcher. false: one person does everything on gamepad1.
    public static boolean TWO_DRIVERS = true;
    // How long the operator's gamepad rumbles when the launcher is ready, in milliseconds.
    public static int READY_RUMBLE_MS = 300;
    // true: camera picture only (HDMI monitor and FTC Dashboard), no AprilTags and no exposure / gain
    // settings, like Camera Test's PLAIN_CAMERA. Aim, distance speed and launch assist then see nothing.
    // Set it in Dashboard BEFORE pressing INIT. Keep it false for matches.
    public static boolean PLAIN_CAMERA = false;

    /*
     * The full telemetry only while the Dashboard is on (practice and tuning). The launch lines
     * (pose OK?, flywheel, saved poses) always show, also in the competition build.
     */
    private static final boolean SHOW_TELEMETRY = RobotSwitches.USE_DASHBOARD;

    /*
     * The robot's parts. They are null until init() creates them, so stop() checks for null:
     * if INIT fails (for example a wrong name in the configuration), the SDK still calls stop().
     */
    private MecanumDriveSubsystem drive = null;
    private IntakeSubsystem intake = null;
    private LauncherSubsystem launcher = null;
    private AprilTagVision vision = null;
    private LaunchAssist launchAssist = null;
    private final CellSightRumble cellRumble = new CellSightRumble();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Send every telemetry line to both the Driver Station and FTC Dashboard.
        if (SHOW_TELEMETRY) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        /*
         * Each subsystem finds its own hardware by the names in the Driver Hub configuration.
         * A part that is switched off in RobotSwitches does not look for its hardware, and all
         * its methods do nothing, so the code below never has to check for a missing part.
         */
        // Bulk reads (SDK): each hub sends all its motor encoders in one message per loop, instead of
        // one message per read, so the loop runs faster.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        // Feed in pulses (about 0.6 s per ball): the Octave launcher check only tested balls 0.42 s or
        // more apart; closer ones can knock each other out of the CELL.
        launcher.setPulseFeed(true);
        vision = new AprilTagVision(hardwareMap, true, PLAIN_CAMERA);
        launchAssist = new LaunchAssist();   // loads the good launch poses saved in earlier runs

        if (SHOW_TELEMETRY) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Switched on", "intake %s, launcher %s, camera %s, dashboard %s, launch assist %s",
                    RobotSwitches.USE_INTAKE, RobotSwitches.USE_LAUNCHER,
                    RobotSwitches.USE_CAMERA, RobotSwitches.USE_DASHBOARD, RobotSwitches.USE_LAUNCH_ASSIST);
            telemetry.addData("IMU", drive.hasImu() ? "found" : "missing: field-centric is off");
        }
        telemetry.addData("Camera hardware", vision.hardwareStatus());
        if (PLAIN_CAMERA) {
            telemetry.addData("Camera mode", "PLAIN (picture only, no AprilTags): set PLAIN_CAMERA = false for matches");
        }
        telemetry.addData("Saved launch poses", launchAssist.successCount());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Pick the alliance before START. WasPressed() also uses up the press, so it doesn't
        // toggle slow mode (X) or save a launch (B) in the first loop after START.
        if (gamepad1.bWasPressed()) {
            vision.setAlliance(AprilTagVision.Alliance.RED);
        }
        if (gamepad1.xWasPressed()) {
            vision.setAlliance(AprilTagVision.Alliance.BLUE);
        }
        if (vision.getAlliance() == null) {
            telemetry.addData("Alliance", "not set (optional): target = the up CELL you point at. Driver B = RED, X = BLUE to use only your HIVE");
        } else {
            telemetry.addData("Alliance", "%s (driver B = RED, X = BLUE to change)", vision.getAlliance());
        }
        telemetry.addData("HIVE up", vision.upCellSummary());
        vision.addReadingLine(telemetry, vision.aimedUpCell());   // range, bearing, yaw, intake to stand
        if (!SHOW_TELEMETRY) {
            telemetry.addData("Camera hardware", vision.hardwareStatus());
        }

        // Lets the drive team check the camera before the match starts.
        if (SHOW_TELEMETRY) {
            vision.addTelemetry(telemetry);
            launchAssist.addTelemetry(telemetry);   // every saved pose, built-in ones first
        }
        telemetry.addData("Saved launch poses", launchAssist.successCount());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        drive.startTeleop();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP.
     * Every loop: 1. look around, 2. the driver drives, 3. the operator runs the launcher and
     * intake, 4. show what is happening.
     */
    @Override
    public void loop() {
        Gamepad driver = gamepad1;
        Gamepad operator = TWO_DRIVERS ? gamepad2 : gamepad1;

        // ---------------- 1. Look: where is the robot, what does the camera see? ----------------
        drive.update();                                          // where the robot is (odometry)
        AprilTagClusterDetection cell = vision.aimedUpCell();    // the up CELL we point at, or null
        launcher.setCell(cell);                                  // the flywheel speed follows its distance
        cellRumble.update(cell != null, driver);                 // blip when a CELL comes into view
        LaunchAssist.Status pose = launchAssist.update(drive, cell, launcher);   // near a good launch pose?
        launchAssist.showOnLed(driver, cell != null);            // LED: green, red, purple or blue

        // ---------------- 2. DRIVER ----------------
        if (driver.xWasPressed()) {
            drive.toggleSlowMode();
        }
        if (driver.yWasPressed()) {
            drive.toggleFieldCentric();      // only when RobotSwitches.USE_FIELD_CENTRIC is true
        }
        if (driver.backWasPressed()) {
            drive.resetHeading();            // field-centric: "forward" = the way the robot faces now
        }

        boolean alignHeld = TWO_DRIVERS ? driver.right_bumper : driver.dpad_up;
        boolean aligning = alignHeld && pose.hasTarget;
        if (aligning) {
            // The robot drives itself to the nearest good launch pose.
            drive.driveToward(pose.forward, pose.strafe, pose.rotate);
        } else if (driver.left_bumper && cell != null) {
            // The sticks drive and strafe, the camera turns the launcher toward the CELL.
            drive.driveWithSticks(driver, vision.aimTurn(vision.launcherBearing(cell)));
        } else {
            drive.driveWithSticks(driver);
        }
        launchAssist.rumbleWhenAligned(driver, aligning);        // "you're there"

        // ---------------- 3. OPERATOR ----------------
        if (operator.rightBumperWasPressed()) {
            launcher.toggleOn();
        }
        if (operator.dpadRightWasPressed()) {
            launcher.adjustTrim(1);          // a bit faster
        }
        if (operator.dpadLeftWasPressed()) {
            launcher.adjustTrim(-1);         // a bit slower
        }
        launcher.run(operator.a);            // off, spinning, or feeding while A is held
        if (launcher.justBecameReady()) {
            operator.rumble(READY_RUMBLE_MS);
        }
        if (operator.aWasPressed() && launcher.isOn()) {
            launchAssist.recordLaunch();     // each A press is one launch in the log
        }
        if (operator.bWasPressed()) {
            launchAssist.saveLastLaunch(operator);   // "that one scored"
        }
        intake.runTeleop(operator.right_trigger - operator.left_trigger, launcher.isFeeding());

        // ---------------- 4. Telemetry ----------------
        vision.addReadingLine(telemetry, cell);   // range, bearing, yaw, intake to stand
        showLaunchLines(aligning);           // always, also in competition
        if (SHOW_TELEMETRY) {
            showTelemetry(cell, driver);
        }
        telemetry.update();
    }

    /*
     * The lines the drive team always sees: which CELL is the target, is the pose right for launching,
     * how many good launch poses are saved, and which way the flywheel spins and how fast.
     */
    private void showLaunchLines(boolean aligning) {
        if (vision.getAlliance() == null) {
            AprilTagClusterDetection target = vision.aimedUpCell();
            telemetry.addData("Target", "%s (alliance not set: the up CELL you point at)",
                    target == null ? "none" : target.metadata.shortName);
        } else {
            telemetry.addData("HIVE up", "%s (alliance %s)", vision.upCellSummary(), vision.getAlliance());
        }
        launchAssist.addDriverLines(telemetry, aligning);
        launcher.addDriverLines(telemetry);
    }

    /*
     * Show what the robot is doing. Lines with a plain number (like "launcher velocity") can be
     * graphed on FTC Dashboard. Only called while the Dashboard is on.
     */
    private void showTelemetry(AprilTagClusterDetection cell, Gamepad driver) {
        telemetry.addData("Drive", drive.getModeText());
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.0f°",
                drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg());
        telemetry.addData("heading", drive.getHeadingDegrees());
        telemetry.addData("Launcher", launcher.getStateText());

        if (vision.isEnabled()) {
            String aim = "off";
            if (driver.left_bumper) {
                aim = cell != null ? "aiming at " + cell.metadata.shortName + " CELL" : "no up CELL seen";
            }
            telemetry.addData("Aim", aim);
            vision.addTelemetry(telemetry);
        }
        launcher.addTelemetry(telemetry);
        drive.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        // Every saved pose, and the last launch as a line to paste into launch_poses_default.csv.
        launchAssist.addTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP. The parts may be null if INIT failed,
     * so each one is checked before we use it.
     */
    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (intake != null) {
            intake.stop();
        }
        if (launcher != null) {
            launcher.stop();
        }
        if (vision != null) {
            vision.close();
        }
    }
}
