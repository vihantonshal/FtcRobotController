package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoLog;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;

import java.util.Locale;

/*
 * BIOBUZZ StarterBot Auto WITHOUT A CAMERA. The same robot, subsystems and step log as
 * "BioBuzz StarterBot Auto", but it never looks for AprilTags: it drives to a FIXED launch spot
 * (odometry, counted from INIT), turns to a fixed heading, launches all four balls at the launcher's
 * one speed (LauncherSubsystem.TARGET_VELOCITY), and PARKS partly in our LOADING ZONE.
 * Use it when the camera is missing, broken, or can't be trusted.
 *
 * START: against our ALLIANCE wall, in the middle of the clear stretch (BioBuzzField
 * RED_START_ALLIANCE_WALL), facing the HIVE. Both alliances use the same numbers.
 *
 * WHY 23 IN FORWARD AND 17.4 IN RIGHT: when AUTO starts the up CELL is red's AUDIENCE CELL and blue's
 * FAR CELL (Competition Manual Figure 10-2). Both are on the robot's RIGHT seen from its alliance wall.
 * BioBuzzField.launchPose() puts the robot in front of that CELL at X = -40 (red), which from the start
 * spot (-63, 2) is 23 in forward and 17.4 in right, facing straight at the HIVE (turn 0). The field is
 * turned 180 degrees for blue, so blue gets the same numbers.
 *
 * INIT CHOICE (gamepad1, during INIT): X = up CELL on the RIGHT (the normal start, the default),
 * B = up CELL on the LEFT (the HIVE has tipped, or the alliance plan says so). The left CELL's launch
 * spot is LAUNCH_FORWARD_IN forward and LAUNCH_LEFT_IN (13.4) left: not a mirror of the right one,
 * because the start spot (Y = 2) is not centered between the CELLS (Y = -15.4 and +15.4).
 *
 * WITHOUT A CAMERA IT CAN'T TELL:
 *  - if the HIVE tips after INIT (the other CELL is then up): it launches at the CELL picked at INIT;
 *  - how far it really is from the CELL: the speed doesn't follow the distance, so put the launch spot
 *    where the launcher scores at TARGET_VELOCITY (the Octave check: robot front about 15 in from the
 *    HIVE stand, 12-32 in worked). Tape-measure it on a real field and tune LAUNCH_* below;
 *  - if odometry slipped: any wheel slip moves the launch spot.
 *
 * TUNE in FTC Dashboard under "BioBuzzStarterBotAutoNoCamera" (copy kept numbers into this file).
 * Steps are logged to /sdcard/FIRST/BioBuzz/auto_log.csv like the camera Auto (up CELL = "no camera").
 */
@Config
@Autonomous(name = "BioBuzz StarterBot Auto (No Camera)", group = "StarterBot")
public class BioBuzzStarterBotAutoNoCamera extends OpMode {

    // ---- The launch spot, from where the robot stood at INIT ----
    // First LAUNCH_FORWARD_IN straight ahead (off the wall, clear of the FLOWER beside it), then
    // LAUNCH_RIGHT_IN to the right. Keep LAUNCH_FORWARD_IN under MAX_FORWARD_IN (the own-side guard).
    public static double LAUNCH_FORWARD_IN = 23;
    public static double LAUNCH_RIGHT_IN = 17.4;
    // The same for an up CELL on the LEFT (INIT button B): LAUNCH_FORWARD_IN ahead, then this far left.
    public static double LAUNCH_LEFT_IN = 13.4;
    // Then turn this many degrees from the start heading (+ = left, - = right) to face the up CELL.
    // 0 = straight at the HIVE, as BioBuzzField.launchPose() does. One for each CELL.
    public static double AIM_TURN_DEG = 0;
    public static double AIM_TURN_LEFT_CELL_DEG = 0;
    public static double DRIVE_TIMEOUT_SECONDS = 5;   // stop driving and launch from wherever it is
    public static double TURN_TIMEOUT_SECONDS = 2;
    public static double TURN_POWER = 0.2;

    // ---- Launch ----
    // The whole time spent launching: 4 balls at about 0.6 s each, plus spin-up and recovery. The
    // roller only feeds while the flywheel is at speed.
    public static double LAUNCH_SECONDS = 8;

    // ---- Park (same spot as the camera Auto) ----
    public static double PARK_FORWARD_IN = 7.5;
    public static double PARK_LEFT_IN = 34;
    public static double PARK_TIMEOUT_SECONDS = 8;

    // ---- Match time and the own-side guard (Competition Manual G402), as in the camera Auto ----
    public static double AUTO_SECONDS = 30;           // PARK starts by AUTO_SECONDS - PARK_TIMEOUT_SECONDS
    public static double MAX_FORWARD_IN = 24;          // never drive further forward than this from INIT

    private enum AutonomousState {
        DRIVE,     // to the launch spot (spinning the flywheel up on the way)
        TURN,      // to the launch heading
        LAUNCH,    // feed all the balls
        PARK,      // into the LOADING ZONE
        COMPLETE
    }

    private AutonomousState autonomousState = AutonomousState.DRIVE;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private double startHeadingDeg = 0;
    private int driveLeg = 0;                 // 0 = going forward, 1 = going sideways
    private boolean upCellLeft = false;       // picked at INIT: X = right (default), B = left
    private boolean parkFacingStart = false;
    private boolean ownSideLimitLogged = false;

    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private AutoLog log;

    @Override
    public void init() {
        if (RobotSwitches.USE_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        // Bulk reads: all motor encoders of a hub in one message per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        launcher.setPulseFeed(true);   // one ball per pulse, waiting for speed in between
        log = new AutoLog(drive, null, launcher);   // null: no camera

        telemetry.addData("Status", "Initialized (NO CAMERA)");
        telemetry.addData("Switched on", "intake %s, launcher %s, dashboard %s",
                RobotSwitches.USE_INTAKE, RobotSwitches.USE_LAUNCHER, RobotSwitches.USE_DASHBOARD);
        telemetry.addData("Feeder", launcher.getFeederText());
    }

    @Override
    public void init_loop() {
        // Which CELL is up: X = right (normal start), B = left (tipped HIVE or alliance plan).
        if (gamepad1.xWasPressed()) {
            upCellLeft = false;
        }
        if (gamepad1.bWasPressed()) {
            upCellLeft = true;
        }
        telemetry.addData("Up CELL", "%s   (gamepad1: X = RIGHT, B = LEFT)", upCellLeft ? "LEFT" : "RIGHT");
        telemetry.addData("Launch spot", "%.1f in forward, %.1f in %s, turn %.0f°", LAUNCH_FORWARD_IN,
                launchSideIn(), upCellLeft ? "left" : "right", aimTurnDeg());
        // IMU check: turn the robot LEFT by hand and this must go UP.
        telemetry.addData("Heading", "%.0f° (turn left by hand: must go up)", drive.getPoseHeadingDeg());
        telemetry.addLine("Start against our ALLIANCE wall, middle of the clear stretch, facing the HIVE.");
        telemetry.update();
    }

    @Override
    public void start() {
        stepTimer.reset();
        log.start();
        startHeadingDeg = drive.getPoseHeadingDeg();
        log.step("INIT", autonomousState.name(), "START pressed, up CELL " + (upCellLeft ? "LEFT" : "RIGHT"));
    }

    @Override
    public void loop() {
        drive.update();   // where the robot is (odometry)

        // Out of time: whatever we are doing, stop and go park.
        boolean beforePark = autonomousState != AutonomousState.PARK
                && autonomousState != AutonomousState.COMPLETE;
        if (beforePark && log.secondsSinceStart() > AUTO_SECONDS - PARK_TIMEOUT_SECONDS) {
            drive.stop();
            launcher.stop();
            if (!log.hasLaunchSpot()) {
                log.launchSpot(startHeadingDeg);
            }
            nextStep(AutonomousState.PARK, String.format(Locale.US, "AUTO deadline (%.0f s)",
                    AUTO_SECONDS - PARK_TIMEOUT_SECONDS));
        }

        switch (autonomousState) {
            case DRIVE:
                launcher.spinUp();   // fast by the time we get there
                if (driveToLaunchSpot()) {
                    drive.stop();
                    nextStep(AutonomousState.TURN, stepTimer.seconds() > DRIVE_TIMEOUT_SECONDS
                            ? "DRIVE timeout" : "at launch spot");
                }
                break;
            case TURN:
                launcher.spinUp();
                boolean turned = drive.turnToHeading(startHeadingDeg + aimTurnDeg(), TURN_POWER);
                if (turned || stepTimer.seconds() > TURN_TIMEOUT_SECONDS) {
                    drive.stop();
                    log.launchSpot(startHeadingDeg);
                    nextStep(AutonomousState.LAUNCH, turned ? "facing the up CELL" : "TURN timeout");
                }
                break;
            case LAUNCH:
                launcher.update(true);   // the roller feeds whenever the flywheel is at speed
                if (!launcher.isEnabled() || stepTimer.seconds() > LAUNCH_SECONDS) {
                    launcher.stop();
                    nextStep(AutonomousState.PARK, launcher.isEnabled() ? "launch time up" : "launcher off");
                }
                break;
            case PARK:
                if (park()) {
                    drive.stop();
                    nextStep(AutonomousState.COMPLETE,
                            stepTimer.seconds() > PARK_TIMEOUT_SECONDS ? "PARK timeout" : "at park spot");
                }
                break;
            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        // The intake helps push the balls along while the roller feeds, and rests otherwise.
        intake.helpFeed(launcher.isFeeding());

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Step time", "%.1f s", stepTimer.seconds());
        log.addTelemetry(telemetry);
        launcher.addDriverLines(telemetry);
        telemetry.addData("Roller", launcher.getRollerText());
        launcher.addTelemetry(telemetry);
        drive.addPositionTelemetry(telemetry);
        if (RobotSwitches.USE_DASHBOARD) {
            drive.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (log != null && log.isStarted()) {
            log.step(autonomousState.name(), "STOP",
                    autonomousState == AutonomousState.COMPLETE ? "AUTO over" : "stopped before COMPLETE");
        }
    }

    // Move to the next step, say why in the step log, and restart the step timer.
    void nextStep(AutonomousState next, String reason) {
        log.step(autonomousState.name(), next.name(), reason);
        autonomousState = next;
        stepTimer.reset();
        ownSideLimitLogged = false;
    }

    /**
     * DRIVE: forward first (off the wall, clear of the FLOWER), then sideways to the launch spot.
     * @return true at the launch spot, or after DRIVE_TIMEOUT_SECONDS.
     */
    boolean driveToLaunchSpot() {
        if (stepTimer.seconds() > DRIVE_TIMEOUT_SECONDS) {
            return true;
        }
        if (driveLeg == 0) {
            if (goTo(LAUNCH_FORWARD_IN, 0)) {
                driveLeg = 1;
            }
            return false;
        }
        // goTo() measures "left": the right CELL's spot is minus LAUNCH_RIGHT_IN.
        return goTo(LAUNCH_FORWARD_IN, upCellLeft ? LAUNCH_LEFT_IN : -LAUNCH_RIGHT_IN);
    }

    // How far sideways the launch spot is (inches, toward the up CELL picked at INIT).
    double launchSideIn() {
        return upCellLeft ? LAUNCH_LEFT_IN : LAUNCH_RIGHT_IN;
    }

    // The turn at the launch spot for the up CELL picked at INIT (+ = left).
    double aimTurnDeg() {
        return upCellLeft ? AIM_TURN_LEFT_CELL_DEG : AIM_TURN_DEG;
    }

    /**
     * PARK: turn back to the start heading, then drive to the park spot.
     * @return true at the park spot, or after PARK_TIMEOUT_SECONDS.
     */
    boolean park() {
        if (stepTimer.seconds() > PARK_TIMEOUT_SECONDS) {
            return true;
        }
        if (!parkFacingStart) {
            parkFacingStart = drive.turnToHeading(startHeadingDeg, TURN_POWER);
            return false;
        }
        return goTo(PARK_FORWARD_IN, PARK_LEFT_IN);
    }

    // One loop of driving to a spot (inches forward, inches left of INIT), facing the start heading,
    // never more than MAX_FORWARD_IN forward (G402). Logs once per step when the guard holds it back.
    boolean goTo(double forwardIn, double leftIn) {
        boolean there = drive.driveToPoint(forwardIn, leftIn, startHeadingDeg, MAX_FORWARD_IN);
        if (drive.hitForwardLimit() && !ownSideLimitLogged) {
            ownSideLimitLogged = true;
            log.step(autonomousState.name(), autonomousState.name(), "own-side limit (G402)");
        }
        return there;
    }
}
