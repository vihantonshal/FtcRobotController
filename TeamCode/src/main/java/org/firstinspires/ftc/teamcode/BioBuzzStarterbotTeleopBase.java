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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.CellSightRumble;
import org.firstinspires.ftc.teamcode.subsystems.LaunchAssist;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;


/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2026-2027 FIRST® Tech Challenge. It leverages a differential/Skid-Steer system for robot mobility,
 * one motor driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * Our team added the camera and LaunchAssist to goBILDA's original (docs/tank-teleop-vision.md):
 *  - LauncherSubsystem runs the launcher and the windmill: it only feeds once the launcher is fast
 *    enough, and the speed follows the distance to the HIVE CELL the camera sees.
 *  - AprilTagVision finds the up CELL the robot points at, to aim at it and set the shot speed.
 *  - LaunchAssist logs every shot, saves good ones, and drives back to a saved good spot.
 * A tank drive can't drive sideways, so LaunchAssist aligns with forward + turn only. This robot's
 * saved poses are its own: launch_poses_tank.csv and /sdcard/FIRST/BioBuzz/launch_log_tank.csv.
 *
 * Controls, all on gamepad1:
 *  AT INIT   B: we are RED   X: we are BLUE (optional). Then only our HIVE counts.
 *  left stick Y: drive          right stick X: turn
 *  right trigger: intake in     left trigger: intake out
 *  right bumper (hold): launch (spins up, feeds once fast enough)
 *  left bumper (hold): turn toward the target CELL
 *  D-pad up (hold): drive to the nearest saved good launch pose
 *  B: the last launch scored, save it     D-pad right/left: shot speed a bit faster/slower
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly. LauncherSubsystem does this for us; tune its numbers in FTC Dashboard.
 */

@Config
@TeleOp(name = "BioBuzz StarterBot Teleop + Vision", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopBase extends OpMode {

    /*
     * Camera on/off for THIS TeleOp only (the mecanum TeleOp and the Auto are not affected).
     * false = launcher test without vision: no camera is looked for, the flywheel runs at
     * LauncherSubsystem.TARGET_VELOCITY + trim, and aim/align do nothing. Change it in FTC Dashboard
     * (BioBuzzStarterbotTeleopBase) before pressing INIT; it is read once at INIT. Back to the value in
     * the code after a restart. RobotSwitches.USE_CAMERA = false still turns the camera off everywhere.
     */
    // LAUNCHER TEST: camera, drive and intake are OFF (only the launcher + windmill). Set all three back to true to drive.
    public static boolean USE_CAMERA = false;
    /*
     * Drive and intake on/off for THIS TeleOp only, the same way (e.g. a launcher test on the bench).
     * false = their motors and servos are not looked for, so they can be unplugged or missing from the
     * configuration, and the sticks / triggers do nothing. USE_INTAKE also needs RobotSwitches.USE_INTAKE.
     */
    public static boolean USE_DRIVE = false;
    public static boolean USE_INTAKE = false;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    // The launcher + windmill, the camera, and launch assist (TeamCode/.../subsystems).
    private LauncherSubsystem launcher = null;
    private AprilTagVision vision = null;
    private LaunchAssist launchAssist = null;

    // USE_DRIVE / USE_INTAKE as they were at INIT, so a Dashboard change during the match can't crash it.
    private boolean driveOn = false;
    private boolean intakeOn = false;

    /*
     * These two variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the arcadeDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftPower;
    double rightPower;

    // Create a variable to set to the intake.
    double intakePower;

    // Launch assist: the last snapshot that saw a CELL (a launch whose camera frame missed the CELL
    // uses it, when it is at most TAG_HOLD_SECONDS old), the LED color, and the "Saved ..." message.
    private static final double TAG_HOLD_SECONDS = 0.5;
    private LaunchAssist.Snapshot lastTagged = null;
    private boolean alignWasOk = false;
    private int ledState = -1;
    private final CellSightRumble cellRumble = new CellSightRumble();   // blips when a CELL comes into view
    private String launchMessage = null;
    private final ElapsedTime launchMessageTimer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // With the Dashboard on, every telemetry line also goes to FTC Dashboard.
        if (RobotSwitches.USE_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        driveOn = USE_DRIVE;
        intakeOn = USE_INTAKE && RobotSwitches.USE_INTAKE;
        if (driveOn) {
            leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        }
        if (intakeOn) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
            rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        }

        // "launcher" and "windmill" (or "windmillServo"), with the launcher's PIDF; see LauncherSubsystem.
        // true = smooth mode: our own gentle speed control, because this flywheel jerks with the hub's PIDF.
        launcher = new LauncherSubsystem(hardwareMap, true);
        // This robot's windmill turns the other way from the mecanum robot's (REVERSE there).
        launcher.setWindmillDirection(DcMotorSimple.Direction.FORWARD);
        // Hold the right bumper to keep launching: the windmill feeds, pauses for the flywheel to recover, feeds again.
        launcher.setPulseFeed(true);
        vision = new AprilTagVision(hardwareMap, USE_CAMERA);
        launchAssist = new LaunchAssist(LaunchAssist.TANK_DEFAULTS_RESOURCE, LaunchAssist.TANK_LOG_PATH);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        if (driveOn) {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        if (driveOn) {
            leftDrive.setZeroPowerBehavior(BRAKE);
            rightDrive.setZeroPowerBehavior(BRAKE);
        }
        if (intakeOn) {
            intake.setZeroPowerBehavior(BRAKE);

            /*
             * set Feeders to an initial value to initialize the servo controller
             */
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);

            /*
             * Much like our drivetrain motors, we set the right intake servo to reverse so that both
             * servos work to pull elements into the intake.
             */
            rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Saved launch poses", "%d, %d built-in", launchAssist.successCount(), launchAssist.defaultCount());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Pick the alliance before START (optional). WasPressed() also uses up the press, so B
        // doesn't save a launch in the first loop after START.
        if (gamepad1.bWasPressed()) {
            vision.setAlliance(AprilTagVision.Alliance.RED);
        }
        if (gamepad1.xWasPressed()) {
            vision.setAlliance(AprilTagVision.Alliance.BLUE);
        }
        if (!vision.isEnabled()) {
            telemetry.addData("Camera", "OFF: launcher test, flywheel at TARGET_VELOCITY + trim (USE_CAMERA in Dashboard)");
        } else if (vision.getAlliance() == null) {
            telemetry.addData("Alliance", "not set (optional): target = the up CELL you point at. B = RED, X = BLUE to use only your HIVE");
        } else {
            telemetry.addData("Alliance", "%s (B = RED, X = BLUE to change)", vision.getAlliance());
            telemetry.addData("HIVE up", vision.upCellSummary());
        }
        if (!driveOn || !intakeOn) {
            telemetry.addData("OFF for this test", "%s%s(Dashboard: BioBuzzStarterbotTeleopBase)",
                    driveOn ? "" : "drive ", intakeOn ? "" : "intake ");
        }
        telemetry.addData("Saved launch poses", "%d, %d built-in", launchAssist.successCount(), launchAssist.defaultCount());
        if (RobotSwitches.USE_DASHBOARD) {
            vision.addTelemetry(telemetry);
            launchAssist.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * The camera's target: the up CELL the robot points at (only our HIVE when an alliance was
         * picked at INIT), or null. The shot speed follows the distance to it.
         */
        AprilTagClusterDetection tag = vision.aimedUpCell();
        launcher.setCellRange(tag != null ? tag.ftcPose.range : Double.NaN);
        cellRumble.update(tag != null, gamepad1);   // blips when the camera finds a CELL, rumbles when lost

        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        // Hold the LEFT bumper: the robot turns toward the target CELL by itself; the stick still drives.
        if (gamepad1.left_bumper && tag != null) {
            rotate = vision.aimTurn(vision.launcherBearing(tag));
        }

        // Launch assist: compare with the nearest saved good launch pose. A tank drive can't
        // strafe, so check(now, false) only fixes distance and direction.
        LaunchAssist.Snapshot now = launchAssist.snapshot(0, 0, 0, tag, launcher);
        if (now.hasTag()) {
            lastTagged = now;
        }
        LaunchAssist.Status launchStatus = launchAssist.check(now, false);
        boolean aligning = gamepad1.dpad_up && launchStatus.hasTarget;
        if (aligning) {
            // Hold D-pad up: the robot drives itself to the saved pose.
            forward = launchStatus.forward;
            rotate = launchStatus.rotate;
        }
        if (aligning && launchStatus.ok && !alignWasOk) {
            gamepad1.rumble(200);   // "you're there"
        }
        alignWasOk = aligning && launchStatus.ok;
        //showPoseOnLed(gamepad1, launchStatus);

        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        if (driveOn) {
            arcadeDrive(forward, rotate);
        }

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        /*
         * The launch() function handles the launcher, the windmill that feeds the elements into the
         * launcher wheel, and the launch log.
         */
        launch(now);

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */
        if (intakeOn) {
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);
        }

        /*
         * Show the target, the launch pose and the launcher on the Driver Station via telemetry.
         */
        showLaunchLines(launchStatus, aligning);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        if (RobotSwitches.USE_DASHBOARD) {
            vision.addTelemetry(telemetry);
            launcher.addTelemetry(telemetry);
            launchAssist.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // The parts are null if INIT failed (for example a wrong name in the configuration).
        if (launcher != null) {
            launcher.stop();
        }
        if (vision != null) {
            vision.close();
        }
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to motors
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void launch(LaunchAssist.Snapshot now) {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. While it is held, LauncherSubsystem spins the launcher
         * up to the shot speed and runs the windmill once the launcher is fast enough to make a
         * successful throw. Let go, and both stop.
         */
        launcher.update(gamepad1.right_bumper);

        /*
         * Each right bumper press is one launch: write it to the log. If the camera missed the CELL in
         * this one frame, use the last snapshot that saw it, when it is at most TAG_HOLD_SECONDS old.
         */
        if (gamepad1.rightBumperWasPressed()) {
            boolean recentTag = lastTagged != null && now.timeMs - lastTagged.timeMs <= TAG_HOLD_SECONDS * 1000;
            launchAssist.recordLaunch(!now.hasTag() && recentTag ? lastTagged : now);
        }

        /*
         * While the windmill feeds, we also add some power to the intake power. This can sometimes
         * help dislodge stuck elements from inside the hopper.
         */
        if (launcher.isFeeding()) {
            //intakePower += 0.5;
        }

        // D-pad right/left: practice trim of the shot speed.
        if (gamepad1.dpadRightWasPressed()) {
            launcher.adjustTrim(1);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            launcher.adjustTrim(-1);
        }

        // B: "that launch scored", save the last launch as a good launch pose.
        if (gamepad1.bWasPressed()) {
            switch (launchAssist.markSuccess()) {
                case SAVED:
                    launchMessage = "Saved good launch #" + launchAssist.successCount();
                    gamepad1.rumble(150);
                    break;
                case NO_TAG:
                    launchMessage = "Not saved: the camera saw no CELL at that launch";
                    break;
                case ALREADY_SAVED:
                    launchMessage = "That launch is already saved";
                    break;
                case OFF:
                    launchMessage = "Launch assist is off (RobotSwitches)";
                    break;
                default:
                    launchMessage = "Nothing to save yet: launch first, then press B";
                    break;
            }
            launchMessageTimer.reset();
        }
    }

    // Gamepad LED: green = pose OK to launch, red = not yet, blue = no target (no CELL in view or none saved).
    private void showPoseOnLed(Gamepad pad, LaunchAssist.Status st) {
        if (!launchAssist.isEnabled()) {
            return;
        }
        int state = !st.hasTarget ? 0 : (st.ok ? 1 : 2);
        if (state == ledState) {
            return;   // only send a new color when it changes
        }
        ledState = state;
        if (state == 1) {
            pad.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (state == 2) {
            pad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            pad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    // The lines the driver always sees: the target, the launch pose, the launcher, saved poses.
    private void showLaunchLines(LaunchAssist.Status st, boolean aligning) {
        if (!vision.isEnabled()) {
            telemetry.addData("Camera", "OFF (launcher test)");
        } else if (vision.getAlliance() == null) {
            AprilTagClusterDetection target = vision.aimedUpCell();
            telemetry.addData("Target", "%s (alliance not set: the up CELL you point at)",
                    target == null ? "none" : target.metadata.shortName);
        } else {
            telemetry.addData("HIVE up", "%s (alliance %s)", vision.upCellSummary(), vision.getAlliance());
        }
        telemetry.addData("Launch pose", (aligning ? "ALIGNING: " : "") + st.text);
        telemetry.addData("Flywheel", "%s %.0f of %.0f ticks/s (%.0f RPM), trim %+.0f",
                launcher.getDirection(), launcher.getVelocity(), launcher.getShootTarget(), launcher.getRpm(),
                launcher.getTrim());
        telemetry.addData("Saved launch poses", "%d, %d built-in (B saves the last launch)",
                launchAssist.successCount(), launchAssist.defaultCount());
        if (launcher.getProblem() != null) {
            telemetry.addData("Launcher", launcher.getProblem());
        }
        if (launchMessage != null && launchMessageTimer.seconds() < 2) {
            telemetry.addLine(launchMessage);
        }
        if (launchAssist.getFileProblem() != null) {
            telemetry.addData("Launch log", launchAssist.getFileProblem());
        }
    }
}
