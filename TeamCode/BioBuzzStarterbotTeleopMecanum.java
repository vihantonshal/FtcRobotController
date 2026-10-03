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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchAssist;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Locale;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels (goBILDA GripForce 104 mm) for the 2026-2027 FIRST® Tech Challenge.
 *
 * Each part of the robot is its own class in the "subsystems" folder, with its own tuning numbers:
 *   MecanumDriveSubsystem  four drive motors + IMU + odometry pose  (drive team)
 *   IntakeSubsystem        intake roller + servos                    (intake team)
 *   LauncherSubsystem      launcher + windmill                       (launcher team)
 *   AprilTagVision         webcam + HIVE tags                        (vision team)
 *   LaunchAssist           log of launches, saved good launch poses, auto-align (launch team)
 *   RobotSwitches          turn a part on or off while it is off the robot
 * This TeleOp only reads the gamepads and tells each part what to do. The Auto uses the same classes.
 *
 * Controls. DRIVER = gamepad1. OPERATOR = gamepad2, or gamepad1 too when TWO_DRIVERS is false
 * (the buttons are chosen so nothing overlaps on one gamepad).
 *  DRIVER    left stick: drive and strafe         right stick X: turn
 *            left bumper (hold): turn toward the up CELL opening (AprilTag cluster)
 *            right bumper (hold): drive to the nearest saved good launch pose
 *                                 (D-pad up instead when TWO_DRIVERS is false)
 *            X: slow mode on/off (for lining up)  Y: field-centric on/off
 *            back/share: reset "forward" to the way the robot faces now
 *  OPERATOR  right trigger: intake in             left trigger: intake out
 *            right bumper: launcher on/off (it keeps spinning; the gamepad rumbles when ready)
 *            A (hold): feed balls; the windmill only feeds once the launcher is fast enough
 *            D-pad right/left: launcher a bit faster/slower (practice trim; the speed also follows
 *                              the distance to the up CELL, see LauncherSubsystem)
 *            B: the last launch scored, save its pose as a good launch pose
 *
 * The Driver Hub ALWAYS shows the launch lines: is the pose OK to launch, flywheel speed and
 * direction, and how many good launch poses are saved. The driver's gamepad LED is green when the
 * pose is OK, red when not, blue when there is no target: no up CELL in view, or no saved pose for
 * it yet (PS4/PS5 gamepads).
 * While RobotSwitches.USE_DASHBOARD is true, the rest of the telemetry also goes to the Driver Station
 * and to FTC Dashboard (http://192.168.43.1:8080/dash), where the numbers can be graphed.
 * Every launch and every saved success is written to /sdcard/FIRST/BioBuzz/launch_log.csv.
 */

@Config
@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    // ---- Driver-feel numbers (owner: drive team) ----
    // true: gamepad2 runs the intake and launcher. false: one person does everything on gamepad1.
    public static boolean TWO_DRIVERS = true;
    // Stick values smaller than this count as 0, so a stick that doesn't center perfectly
    // doesn't make the robot creep.
    public static double DEADBAND = 0.05;
    // Slow mode multiplies every drive command by this (0.4 = 40% speed).
    public static double SLOW_MODE_SCALE = 0.4;
    // How fast a drive command may change, per second. 4 means 0 to full speed in 0.25 s:
    // smooth enough that the robot doesn't jerk or tip, quick enough to feel direct.
    public static double MAX_ACCEL_PER_SEC = 4.0;
    // Start in field-centric mode when the IMU is found.
    public static boolean FIELD_CENTRIC_AT_START = true;
    // How long the gamepad rumbles when the launcher is ready, in milliseconds.
    public static int READY_RUMBLE_MS = 300;

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

    // Modes the drivers switch with a button press.
    private boolean fieldCentric = false;
    private boolean slowMode = false;
    private boolean launcherOn = false;
    private boolean launcherWasReady = false;


    // Launch assist state.
    private boolean wasFeeding = false;           // to catch the moment feeding starts
    private boolean alignWasOk = false;           // to rumble once when auto-align gets there
    private int ledState = -1;                    // last gamepad LED color we set
    private String launchMessage = null;          // "Saved good launch #3", shown for 2 seconds
    private final ElapsedTime launchMessageTimer = new ElapsedTime();

    // Smoothed drive commands and the timer that measures each loop.
    private double forwardCmd = 0;
    private double strafeCmd = 0;
    private double rotateCmd = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

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
        vision = new AprilTagVision(hardwareMap);
        launchAssist = new LaunchAssist();   // loads the good launch poses saved in earlier runs

        fieldCentric = FIELD_CENTRIC_AT_START && drive.hasImu();

        if (SHOW_TELEMETRY) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Switched on", "intake %s, launcher %s, camera %s, dashboard %s",
                    RobotSwitches.USE_INTAKE, RobotSwitches.USE_LAUNCHER,
                    RobotSwitches.USE_CAMERA, RobotSwitches.USE_DASHBOARD);
            telemetry.addData("IMU", drive.hasImu() ? "found" : "missing: field-centric is off");
        }
        telemetry.addData("Saved launch poses", launchAssist.successCount());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Lets the drive team check the camera before the match starts.
        if (SHOW_TELEMETRY) {
            vision.addTelemetry(telemetry);
        }
        telemetry.addData("Saved launch poses", launchAssist.successCount());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        loopTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // How long the last loop took, capped so a slow loop can't cause a big jump.
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        Gamepad driver = gamepad1;
        Gamepad operator = TWO_DRIVERS ? gamepad2 : gamepad1;

        // Updates the odometry pose.
        drive.update();

        // ---------------- Driver: mode buttons (one toggle per press) ----------------
        // xWasPressed() (SDK) is true once per press, so holding the button toggles only once.
        if (driver.xWasPressed()) {
            slowMode = !slowMode;
        }
        if (driver.yWasPressed() && drive.hasImu()) {
            fieldCentric = !fieldCentric;
        }
        if (driver.backWasPressed()) {
            // Point the robot away from the driver, then press: that direction is now "forward".
            drive.resetHeading();
        }

        // ---------------- Driver: sticks ----------------
        /*
         * Moving the left stick forward gives a negative number on most gamepads, so we flip it.
         * Small stick values count as 0 (deadband), so the robot doesn't creep.
         */
        double forward = deadband(-driver.left_stick_y);
        double strafe = deadband(driver.left_stick_x);
        double rotate = deadband(driver.right_stick_x);

        /*
         * While the driver holds the LEFT bumper and the camera sees a HIVE tag, the robot turns
         * toward the tag by itself instead. The driver can still drive and strafe.
         */
        AprilTagClusterDetection tag = vision.closestTag();   // the up CELL; null when no up CELL is seen or the camera is off
        // The flywheel speed follows the distance to the up CELL (LauncherSubsystem, distance-based speed).
        launcher.setCellRange(tag != null ? tag.ftcPose.range : Double.NaN);
        if (driver.left_bumper && tag != null) {
            rotate = vision.aimTurn(tag.ftcPose.bearing);
        }

        // ---------------- Launch assist: compare with the nearest good launch pose ----------------
        LaunchAssist.Snapshot now = launchAssist.snapshot(drive, tag, launcher);
        LaunchAssist.Status launchStatus = launchAssist.check(now);
        boolean alignHeld = TWO_DRIVERS ? driver.right_bumper : driver.dpad_up;
        boolean aligning = alignHeld && launchStatus.hasTarget;
        if (aligning) {
            // The robot drives itself to the saved pose while the button is held.
            forward = launchStatus.forward;
            strafe = launchStatus.strafe;
            rotate = launchStatus.rotate;
        } else if (slowMode) {
            forward *= SLOW_MODE_SCALE;
            strafe *= SLOW_MODE_SCALE;
            rotate *= SLOW_MODE_SCALE;
        }
        if (aligning && launchStatus.ok && !alignWasOk) {
            driver.rumble(200);   // "you're there"
        }
        alignWasOk = aligning && launchStatus.ok;
        showPoseOnLed(driver, launchStatus);

        // Smooth the commands so the robot speeds up and slows down gently.
        double step = MAX_ACCEL_PER_SEC * dt;
        forwardCmd = approach(forwardCmd, forward, step);
        strafeCmd = approach(strafeCmd, strafe, step);
        rotateCmd = approach(rotateCmd, rotate, step);

        // Auto-align commands are robot-centric, so field-centric is skipped while aligning.
        if (fieldCentric && !aligning) {
            drive.driveFieldCentric(forwardCmd, strafeCmd, rotateCmd);
        } else {
            drive.drive(forwardCmd, strafeCmd, rotateCmd);
        }

        // ---------------- Operator: launcher ----------------
        // The right bumper turns the launcher on and off; it keeps spinning while on.
        if (operator.rightBumperWasPressed() && launcher.isEnabled()) {
            launcherOn = !launcherOn;
        }

        // D-pad right/left: a bit faster/slower (practice trim, shown on the "Shot speed" line).
        if (operator.dpadRightWasPressed()) {
            launcher.adjustTrim(1);
        }
        if (operator.dpadLeftWasPressed()) {
            launcher.adjustTrim(-1);
        }

        if (!launcherOn) {
            launcher.stop();
            launcherWasReady = false;
        } else if (operator.a) {
            launcher.update(true);    // feed; the windmill waits until the launcher is fast enough
        } else {
            launcher.spinUp();        // keep spinning, no feeding
        }

        /*
         * Rumble once when the launcher first reaches speed after being turned on, so the operator
         * knows to press A. launcherWasReady stays true until the launcher is turned off, so the
         * small speed dip after each shot doesn't rumble again.
         */
        boolean ready = launcherOn && launcher.isAtSpeed();
        if (ready && !launcherWasReady) {
            operator.rumble(READY_RUMBLE_MS);
            launcherWasReady = true;
        }

        // ---------------- Operator: log launches and save good ones ----------------
        // Every time the windmill starts feeding counts as a launch: write it to the log.
        boolean feeding = launcher.isFeeding();
        if (feeding && !wasFeeding) {
            launchAssist.recordLaunch(launchAssist.snapshot(drive, tag, launcher));
        }
        wasFeeding = feeding;

        // B: "that launch scored", save the last launch's pose as a good launch pose.
        if (operator.bWasPressed()) {
            if (launchAssist.markSuccess()) {
                launchMessage = "Saved good launch #" + launchAssist.successCount();
                operator.rumble(150);
            } else {
                launchMessage = "Nothing to save yet: launch first, then press B";
            }
            launchMessageTimer.reset();
        }

        // ---------------- Operator: intake ----------------
        /*
         * Intake power = right trigger minus left trigger (-1 to 1). While the windmill feeds, we
         * add 0.5 to help push stuck balls along. The intake keeps the power inside -1..1.
         */
        double intakePower = operator.right_trigger - operator.left_trigger;
        if (feeding) {
            intakePower += 0.5;
        }
        intake.run(intakePower, intakePower);

        // ---------------- Telemetry ----------------
        showLaunchLines(launchStatus, aligning);   // always, also in competition
        if (SHOW_TELEMETRY) {
            showTelemetry(tag, driver);
        }
        telemetry.update();
    }

    // Stick values smaller than DEADBAND count as 0.
    private static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }

    // Move "current" toward "target" by at most "step".
    private static double approach(double current, double target, double step) {
        return current + Range.clip(target - current, -step, step);
    }

    // Driver's gamepad LED: green = pose OK to launch, red = not yet, blue = no target (no CELL in view or none saved).
    private void showPoseOnLed(Gamepad driver, LaunchAssist.Status st) {
        int state = !st.hasTarget ? 0 : (st.ok ? 1 : 2);
        if (state == ledState) {
            return;   // only send a new color when it changes
        }
        ledState = state;
        if (state == 1) {
            driver.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (state == 2) {
            driver.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            driver.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    /*
     * The lines the drive team always sees: is the pose right for launching, which way the flywheel
     * spins and how fast, and how many good launch poses are saved.
     */
    private void showLaunchLines(LaunchAssist.Status st, boolean aligning) {
        telemetry.addData("Launch pose", (aligning ? "ALIGNING: " : "") + st.text);
        telemetry.addData("Flywheel", "%s %.0f of %.0f ticks/s (%.0f RPM)",
                launcher.getDirection(), launcher.getVelocity(), launcher.getShootTarget(), launcher.getRpm());
        double range = launcher.getCellRange();
        telemetry.addData("Shot speed", "%s, trim %+.0f (operator D-pad left/right)",
                Double.isNaN(range) ? "no CELL seen yet" : String.format(Locale.US, "for a CELL %.0f in away", range),
                launcher.getTrim());
        if (launcherOn && "REVERSE".equals(launcher.getDirection())) {
            telemetry.addLine("WARNING: the flywheel spins BACKWARD");
        }
        telemetry.addData("Saved launch poses", "%d (operator B saves the last launch)", launchAssist.successCount());
        if (launchMessage != null && launchMessageTimer.seconds() < 2) {
            telemetry.addLine(launchMessage);
        }
        if (launchAssist.getFileProblem() != null) {
            telemetry.addData("Launch log", launchAssist.getFileProblem());
        }
    }

    /*
     * Show what the robot is doing. Lines with a plain number (like "launcher velocity") can be
     * graphed on FTC Dashboard. Only called while the Dashboard is on.
     */
    private void showTelemetry(AprilTagClusterDetection tag, Gamepad driver) {
        String driveMode = fieldCentric ? "field-centric" : (drive.hasImu() ? "robot-centric" : "robot-centric (no IMU)");
        telemetry.addData("Drive", "%s%s", driveMode, slowMode ? ", SLOW" : "");
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.0f°",
                drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg());
        telemetry.addData("heading", drive.getHeadingDegrees());

        String launcherState = "off";
        if (launcherOn) {
            launcherState = launcher.isFeeding() ? "feeding" : (launcher.isAtSpeed() ? "ready" : "spinning up");
        }
        telemetry.addData("Launcher", launcherState);

        if (vision.isEnabled()) {
            String aim = "off";
            if (driver.left_bumper) {
                aim = tag != null ? "aiming at " + tag.metadata.shortName + " CELL" : "no up CELL seen";
            }
            telemetry.addData("Aim", aim);
            vision.addTelemetry(telemetry);
        }
        launcher.addTelemetry(telemetry);
        drive.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
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
