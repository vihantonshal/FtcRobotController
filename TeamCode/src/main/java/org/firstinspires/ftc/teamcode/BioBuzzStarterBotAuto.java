/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;


/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2026-2027 FIRST® Tech Challenge season BIOBUZZ™. It uses the same subsystem classes as the
 * TeleOp (see the "subsystems" folder), so the hardware names and tuning numbers live in one place.
 *
 * This robot starts up against our ALLIANCE wall facing the HIVE. It first drives to a spot where
 * the camera can read the up CELL's AprilTags (LOOK), turns to face the up CELL (while the
 * launcher spins up), launches all four projectiles, and then PARKS partly in our LOADING ZONE.
 * If the camera still can't see the up CELL, it SEARCHES: it turns slowly left and right, in
 * place, until the up CELL comes into view. If the HIVE has TIPPED, the other CELL is up and can't
 * be seen from the LOOK spot, so after a few seconds it tries the mirror LOOK spot on the LEFT.
 *
 * WHY LOOK: each CELL's AprilTags are on its BOTTOM face (Competition Manual 9.9), so when a CELL
 * tips up its tags face down and out, toward the end of the field beyond it. At the start of
 * AUTO the up CELL is the audience one for red and the far one for blue: both on the robot's
 * RIGHT when it faces the HIVE from its alliance wall. From the middle of the wall the camera
 * would see those tags from behind, so LOOK moves forward off the wall and then to the right.
 *
 * LOOK and PARK use odometry (MecanumDriveSubsystem's pose, counted from INIT), so PARK drives
 * from wherever the robot really is after launching to the same park spot.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name="BioBuzz StarterBot Auto", group="StarterBot")
//@Disabled
public class BioBuzzStarterBotAuto extends OpMode
{
    /*
     * The plan for this Auto. Powers are 0-1, with 1 being full speed.
     * AIM and SEARCH share FIND_TIMEOUT_SECONDS, counted from the end of LOOK: when it runs out,
     * the robot turns back to its start heading and launches anyway. LAUNCH feeds for LAUNCH_SECONDS.
     */
    final double FIND_TIMEOUT_SECONDS = 6;
    final double LAUNCH_SECONDS = 10;

    /*
     * The whole AUTO period. PARK must start by AUTO_SECONDS - PARK_TIMEOUT_SECONDS (22 s), so
     * whatever step the robot is in then (LOOK, AIM, SEARCH or LAUNCH), it stops and goes to PARK.
     */
    final double AUTO_SECONDS = 30;

    /*
     * LOOK: where to see the up CELL's tags from, measured from where the robot stood at INIT.
     * First LOOK_FORWARD_IN straight ahead (off the wall, clear of the FLOWER beside it), then
     * LOOK_RIGHT_IN to the right. The same move works for red and blue (see the top of the file).
     * Set LOOK_RIGHT_IN = 0 and LOOK_FORWARD_IN = 0 to aim from the start spot instead.
     */
    final double LOOK_FORWARD_IN = 18;
    final double LOOK_RIGHT_IN = 30;
    final double LOOK_TIMEOUT_SECONDS = 4;
    private int lookLeg = 0;               // 0 = going forward, 1 = going sideways

    /*
     * OTHER SIDE: if the HIVE has TIPPED (for example our partner filled the up CELL before we
     * launched), the OTHER CELL is up and its tags face the other end of the field, so from the
     * LOOK spot the camera only sees them from behind. If no up CELL is in view
     * OTHER_SIDE_AFTER_SECONDS after LOOK, the robot drives to the mirror LOOK spot,
     * LOOK_RIGHT_IN to the LEFT of where it started, and aims from there (once).
     * The sideways move is twice as long, so it gets OTHER_LOOK_TIMEOUT_SECONDS.
     */
    final double OTHER_SIDE_AFTER_SECONDS = 3;
    final double OTHER_LOOK_TIMEOUT_SECONDS = 5;
    private boolean lookingOtherSide = false;

    /*
     * SEARCH: turn in place up to SEARCH_ANGLE_DEG to the left and to the right of the start
     * heading. Slow, so the camera picture (which is always a little late) keeps up.
     */
    final double SEARCH_ANGLE_DEG = 45;
    final double SEARCH_TURN_POWER = 0.2;
    final double HEADING_TOLERANCE_DEG = 3;
    // After FIND_TIMEOUT_SECONDS, how long SEARCH may take to turn back to the start heading. If
    // the robot can't turn (stuck on something), it launches from wherever it is.
    final double TURN_BACK_TIMEOUT_SECONDS = 2;

    /*
     * PARK: where to stop, measured from where the robot stood at INIT (inches forward, inches to
     * the left), for a start against our ALLIANCE wall facing the HIVE. From BioBuzzField:
     * RED_PARK (-58, 36) minus RED_START_ALLIANCE_WALL (-63, 2) = 5 in forward, 34 in left. The
     * field is turned 180 degrees for blue, so the LOADING ZONE is on the robot's left for both.
     * Measure these on a real field: PARK scores when the robot is at least partially in the
     * LOADING ZONE (Competition Manual 10.5.4).
     * PARK_FORWARD_IN is 7.5, not 5, so the robot stays off the wall: a partner that doesn't show up
     * has its 4 pre-loaded POLLEN placed "in approximately the center of the LOADING ZONE against the
     * perimeter wall" (Manual 10.3.1). The robot starts with its back against the wall, so at 7.5 in
     * its back edge is 7.5 in from the wall: still 3.5 in inside the zone's 11 in tape line, and one
     * POLLEN width plus a few inches clear of POLLEN lying against the wall (5 in left almost none).
     */
    final double PARK_FORWARD_IN = 7.5;
    final double PARK_LEFT_IN = 34;
    // How LOOK and PARK drive to a spot (driveToward()).
    final double PARK_TOLERANCE_IN = 1.5;
    final double PARK_MAX_POWER = 0.4;
    final double PARK_MIN_POWER = 0.1;     // enough to keep moving when the spot is close
    final double PARK_GAIN = 0.05;         // power per inch still to go
    final double PARK_HEADING_GAIN = 0.01; // turn power per degree, to keep facing the HIVE
    final double PARK_TIMEOUT_SECONDS = 8; // stop wherever it is, so AUTO never runs out mid-move
    private boolean parkFacingStart = false;

    /*
     * OWN-SIDE GUARD (Competition Manual G402): during AUTO, columns A-C are red's side of the
     * FIELD and D-F blue's, so the line between them is 72 in from each ALLIANCE wall. driveToward()
     * never drives the robot more than MAX_FORWARD_IN forward of where it stood at INIT; past it,
     * it may still slide sideways or back up, but not go further toward the middle. From the
     * ALLIANCE wall the robot's center starts 63 in from that line. At full speed it takes about
     * 6 in to stop (measured in the simulator), so 24 in keeps even its corners (12.7 in from the
     * center when turned) about 20 in clear. Red and blue both start against their own wall facing
     * the HIVE, so "forward" points at the middle for both: no alliance setting needed.
     * Keep it above LOOK_FORWARD_IN and PARK_FORWARD_IN, and under 40.
     */
    final double MAX_FORWARD_IN = 24;
    private boolean ownSideLimitLogged = false;   // one step-log row per step when the guard acts
    // Where the robot launched from, shown on the Driver Hub from LAUNCH to the end of AUTO.
    private String launchSpot = null;

    // A timer that restarts at the beginning of every step of our auto.
    private final ElapsedTime stepTimer = new ElapsedTime();
    // A timer from the end of LOOK, for FIND_TIMEOUT_SECONDS.
    private final ElapsedTime findTimer = new ElapsedTime();
    // A timer from START, for AUTO_SECONDS.
    private final ElapsedTime autoTimer = new ElapsedTime();
    // Where the robot faced at START (degrees, + = left), and which way the search turns next.
    private double startHeadingDeg = 0;
    private double searchTargetDeg = 0;

    /*
     * STEP LOG: every step change (and START and STOP) is one row in /sdcard/FIRST/BioBuzz/auto_log.csv
     * on the Control Hub, with the reason, the time since START and where odometry says the robot is.
     * Download it after a match:  adb pull /sdcard/FIRST/BioBuzz/auto_log.csv
     * The same steps also stay on the Driver Hub (the "Step 1", "Step 2"... lines) until the next INIT.
     * "run" is the date and time START was pressed, so the rows of one match belong together.
     */
    public static final String LOG_PATH = "BioBuzz/auto_log.csv";
    private static final String LOG_HEADER = "run,t_s,from,to,reason,x_in,y_in,heading_deg,up_cell,launcher_tps";
    private final File logFile = new File(AppUtil.ROOT_FOLDER, LOG_PATH);
    private final List<String> stepHistory = new ArrayList<>();
    private String runId = "";
    private String logProblem = null;   // shown on the Driver Hub if the file can't be written

    /*
     * TAG LOG: did the camera see the HIVE's AprilTags, and which? One row in
     * /sdcard/FIRST/BioBuzz/auto_tags.csv each time what the camera sees changes (another CELL, more or
     * fewer of its tags, UP or down, or nothing), and at most once a second while it stays the same.
     * Example row: run, 2.97, AIM, RED AUDIENCE IDs 34-37 2/4 tags UP 35 in 2 deg left
     * SDK 12 reports a HIVE cluster's 4 tags together, so a row shows the cluster's ID range and how
     * many of its tags were found, not single tag numbers. "none" = no tag seen.
     */
    public static final String TAG_LOG_PATH = "BioBuzz/auto_tags.csv";
    private static final String TAG_LOG_HEADER = "run,t_s,state,camera_sees";
    private final File tagLogFile = new File(AppUtil.ROOT_FOLDER, TAG_LOG_PATH);
    private final ElapsedTime tagLogTimer = new ElapsedTime();
    private String lastSeenKey = null;      // what was seen at the last tag-log row (without distances)
    private String cameraSees = "";         // the Driver Hub's "Camera sees" line

    // The robot's parts. Switched-off parts (RobotSwitches) do nothing.
    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private AprilTagVision vision;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "AutonomousState." It reflects the current state of our auto.
     * It starts at LOOK, and we can use higher level code to cycle through these states.
     * This allows us to write functions and autonomous routines in a way that avoids
     * loops within loops, and "waits."
     * AIM and SEARCH can switch back and forth: AIM goes to SEARCH when it can't see the up CELL,
     * and SEARCH goes back to AIM as soon as it does.
     */
    private enum AutonomousState {
        LOOK,
        AIM,
        SEARCH,
        LAUNCH,
        PARK,
        COMPLETE
    }

    private AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        // Send every telemetry line to both the Driver Station and FTC Dashboard.
        if (RobotSwitches.USE_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        // The first step of our autonomous state machine.
        autonomousState = AutonomousState.LOOK;

        // Bulk reads (SDK): each hub sends all its motor encoders in one message per loop, instead of
        // one message per read, so the loop runs faster.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Each subsystem finds its own hardware and sets up its motors and PIDF numbers.
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        vision = new AprilTagVision(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Switched on", "intake %s, launcher %s, camera %s, dashboard %s",
                RobotSwitches.USE_INTAKE, RobotSwitches.USE_LAUNCHER,
                RobotSwitches.USE_CAMERA, RobotSwitches.USE_DASHBOARD);
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        // Lets the drive team check that the camera sees a HIVE tag before the match starts.
        vision.addTelemetry(telemetry);
        /*
         * IMU check: turn the robot LEFT by hand and this must go UP. If it goes down or barely
         * moves, MecanumDriveSubsystem.HUB_LOGO / HUB_USB don't match how the Control Hub is
         * mounted, and SEARCH and PARK will turn the wrong way.
         */
        telemetry.addData("Heading", "%.0f° (turn left by hand: must go up)", drive.getPoseHeadingDeg());
        telemetry.update();
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        stepTimer.reset();
        findTimer.reset();
        autoTimer.reset();
        startHeadingDeg = drive.getPoseHeadingDeg();
        searchTargetDeg = startHeadingDeg + SEARCH_ANGLE_DEG;   // look left first
        runId = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
        logStep("INIT", autonomousState.name(), "START pressed");
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        // Updates the odometry pose.
        drive.update();
        // The flywheel speed follows the distance to the up CELL (LauncherSubsystem, distance-based speed).
        AprilTagClusterDetection seenCell = vision.closestTag();
        launcher.setCellRange(seenCell != null ? seenCell.ftcPose.range : Double.NaN);

        // The intake only helps while the windmill feeds, so it starts from 0 each loop.
        double intakePower = 0;

        // Out of time: whatever we are doing, stop and go park, so PARK gets its full time.
        boolean beforePark = autonomousState != AutonomousState.PARK
                && autonomousState != AutonomousState.COMPLETE;
        if (beforePark && autoTimer.seconds() > AUTO_SECONDS - PARK_TIMEOUT_SECONDS) {
            drive.stop();
            launcher.stop();
            if (launchSpot == null) {
                recordLaunchSpot();   // shows where it stopped, though it never launched
            }
            nextStep(AutonomousState.PARK, String.format(Locale.US, "AUTO deadline (%.0f s)",
                    AUTO_SECONDS - PARK_TIMEOUT_SECONDS));
        }

        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We end each case with "break" to skip out of checking the rest of the members of the enum.
         */
        switch (autonomousState) {
            case LOOK:
                // Spin the flywheel up on the way, so it is fast when we get to LAUNCH.
                launcher.spinUp();
                if (look()) {
                    drive.stop();
                    findTimer.reset();     // AIM and SEARCH get their full time from here
                    nextStep(AutonomousState.AIM, stepTimer.seconds() > lookTimeoutSeconds() ? "LOOK timeout"
                            : lookingOtherSide ? "at other LOOK spot" : "at LOOK spot");
                }
                break;
            case AIM:
                // Start the flywheel now so it is fast when we get to LAUNCH. No feeding yet.
                launcher.spinUp();
                if (tryOtherSide()) {
                    break;
                }
                if (aim()) {
                    drive.stop();
                    // aim() turned the wheels, so start the next move from 0.
                    drive.resetEncoders();
                    recordLaunchSpot();
                    nextStep(AutonomousState.LAUNCH, !vision.isEnabled() ? "camera off"
                            : findTimer.seconds() > FIND_TIMEOUT_SECONDS ? "find timeout" : "aimed at up CELL");
                } else if (vision.closestTag() == null && stepTimer.seconds() > 0.5) {
                    // No up CELL in view for half a second: go and look for it.
                    nextStep(AutonomousState.SEARCH, "up CELL not in view");
                }
                break;
            case SEARCH:
                launcher.spinUp();
                if (tryOtherSide()) {
                    break;
                }
                if (search()) {
                    drive.stop();
                    drive.resetEncoders();
                    // search() is done: it found the up CELL, or ran out of time and turned back.
                    if (vision.closestTag() != null && findTimer.seconds() <= FIND_TIMEOUT_SECONDS) {
                        nextStep(AutonomousState.AIM, "up CELL in view");
                    } else {
                        recordLaunchSpot();
                        nextStep(AutonomousState.LAUNCH,
                                findTimer.seconds() > FIND_TIMEOUT_SECONDS + TURN_BACK_TIMEOUT_SECONDS
                                        ? "find timeout; turn-back timeout" : "find timeout; turned back");
                    }
                }
                break;
            case LAUNCH:
                launcher.update(true);
                if (launcher.isFeeding()) {
                    intakePower = 0.5;
                }
                // With the launcher switched off there is nothing to launch, so move straight on.
                if (!launcher.isEnabled() || stepTimer.seconds() > LAUNCH_SECONDS) {
                    launcher.stop();
                    intakePower = 0;
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

        // In the Auto only the intake roller helps; the corner servos stay still.
        intake.run(intakePower, 0);

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it runs once every loop,
         * no matter what state our robot is in.
         */
        logTagsIfChanged();
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Step time", "%.1f s", stepTimer.seconds());
        telemetry.addData("Camera sees", cameraSees);
        if (launchSpot != null) {
            telemetry.addData("Launched", launchSpot);
        }
        addStepTelemetry();
        vision.addTelemetry(telemetry);
        launcher.addTelemetry(telemetry);
        drive.addPositionTelemetry(telemetry);
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
        // drive and vision are null if INIT failed (for example a wrong name in the configuration).
        // runId is empty if STOP came before START: nothing ran, so there is nothing to log.
        if (drive != null && !runId.isEmpty()) {
            logStep(autonomousState.name(), "STOP",
                    autonomousState == AutonomousState.COMPLETE ? "AUTO over" : "stopped before COMPLETE");
        }
        if (vision != null) {
            vision.close();
        }
    }

    // Move to the next step of the state machine, say why in the step log, and restart the step timer.
    void nextStep(AutonomousState next, String reason) {
        logStep(autonomousState.name(), next.name(), reason);
        autonomousState = next;
        stepTimer.reset();
        ownSideLimitLogged = false;
    }

    /**
     * Adds one row to the step log file and one line to the Driver Hub's step list. Called only when
     * the step changes (about 7 times a match), so writing the file does not slow the loop down.
     */
    void logStep(String from, String to, String reason) {
        double t = autoTimer.seconds();
        AprilTagClusterDetection cell = vision.closestTag();
        String upCell = !vision.isEnabled() ? "camera off" : cell == null ? "none" : cell.metadata.shortName;
        stepHistory.add(String.format(Locale.US, "%5.1f s  %s -> %s: %s", t, from, to, reason));
        String row = String.format(Locale.US, "%s,%.2f,%s,%s,%s,%.1f,%.1f,%.1f,%s,%.0f",
                runId, t, from, to, reason, drive.getPoseX(), drive.getPoseY(), drive.getPoseHeadingDeg(),
                upCell, launcher.getVelocity());
        appendRow(logFile, LOG_HEADER, row, "step log");
    }

    /**
     * One row in the tag log when what the camera sees changes, or once a second while it stays the
     * same, so the file shows when the camera saw the HIVE's tags and when it saw nothing.
     */
    void logTagsIfChanged() {
        String key = vision.seenSummary(false);
        cameraSees = vision.seenSummary(true);
        if (key.equals(lastSeenKey) && tagLogTimer.seconds() < 1) {
            return;
        }
        lastSeenKey = key;
        tagLogTimer.reset();
        appendRow(tagLogFile, TAG_LOG_HEADER, String.format(Locale.US, "%s,%.2f,%s,%s",
                runId, autoTimer.seconds(), autonomousState, cameraSees), "tag log");
    }

    // Adds one line to a CSV file on the Control Hub (with the header if the file is new).
    void appendRow(File file, String header, String row, String what) {
        try {
            File dir = file.getParentFile();
            if (dir != null && !dir.exists() && !dir.mkdirs()) {
                logProblem = "can't create " + dir;
                return;
            }
            boolean isNew = !file.exists();
            try (FileWriter w = new FileWriter(file, true)) {
                if (isNew) {
                    w.write(header + "\n");
                }
                w.write(row + "\n");
            }
            logProblem = null;
        } catch (IOException e) {
            logProblem = what + ": " + e.getMessage();
        }
    }

    // The steps so far, one line each, so the drive team can read the whole match on the Driver Hub.
    void addStepTelemetry() {
        for (int i = 0; i < stepHistory.size(); i++) {
            telemetry.addData("Step " + (i + 1), stepHistory.get(i));
        }
        if (logProblem != null) {
            telemetry.addData("Log NOT saved", logProblem);
        }
    }

    /**
     * The AprilTag decision: turns the robot toward the up CELL.
     * @return true when the robot points at the up CELL (within AprilTagVision.AIM_TOLERANCE_DEG),
     * when the camera is switched off, or when FIND_TIMEOUT_SECONDS has run out. While no up CELL
     * is in view it holds still and returns false; the AIM case then switches to SEARCH.
     */
    boolean aim() {
        if (!vision.isEnabled() || findTimer.seconds() > FIND_TIMEOUT_SECONDS) {
            return true;  // no camera, or out of time: launch from here
        }
        AprilTagDetection tag = vision.closestTag();
        if (tag == null) {
            drive.stop();
            return false;
        }
        if (vision.isAimedAt(tag)) {
            drive.stop();
            return true;
        }
        // Turn in place toward the up CELL: the same rule as the TeleOp's left bumper.
        drive.drive(0, 0, vision.aimTurn(tag.ftcPose.bearing));
        return false;
    }

    /**
     * Looks for the up CELL by turning in place: to SEARCH_ANGLE_DEG left of the start heading,
     * then to SEARCH_ANGLE_DEG right, then left again, until the camera sees it.
     * @return true when the up CELL is in view, or, after FIND_TIMEOUT_SECONDS, once the robot has
     * turned back to its start heading (so it never launches from the middle of a sweep).
     */
    boolean search() {
        if (findTimer.seconds() > FIND_TIMEOUT_SECONDS) {
            // Turn back to the start heading, but never for longer than TURN_BACK_TIMEOUT_SECONDS.
            return turnToward(startHeadingDeg)
                    || findTimer.seconds() > FIND_TIMEOUT_SECONDS + TURN_BACK_TIMEOUT_SECONDS;
        }
        if (vision.closestTag() != null) {
            return true;
        }
        if (turnToward(searchTargetDeg)) {
            // Reached one side: sweep to the other side.
            searchTargetDeg = searchTargetDeg > startHeadingDeg
                    ? startHeadingDeg - SEARCH_ANGLE_DEG : startHeadingDeg + SEARCH_ANGLE_DEG;
        }
        return false;
    }

    /**
     * Remembers where the robot launches from, for the "Launched" telemetry line: which CELL it
     * aimed at, how far it turned from its start heading (+ = left), and where odometry says it is.
     * The cluster name gives the HIVE end on both alliances: SCORING = far CELL, AUDIENCE = audience CELL.
     * Example: "RED SCORING (far CELL), turned 15° left, at (0.3, -0.1) in".
     */
    void recordLaunchSpot() {
        AprilTagClusterDetection cell = vision.closestTag();
        String target;
        if (!vision.isEnabled()) {
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

    /**
     * Drives to the park spot (PARK_FORWARD_IN, PARK_LEFT_IN from where the robot stood at INIT).
     * First it turns back to the start heading, then every loop it asks odometry where the robot
     * really is, works out how far the spot is ahead and to the left, and drives straight at it,
     * slower as it gets close. So it does not matter where AIM and SEARCH left the robot.
     * @return true when the robot is within PARK_TOLERANCE_IN of the spot, or after
     * PARK_TIMEOUT_SECONDS (it then stops wherever it is).
     */
    boolean park() {
        if (stepTimer.seconds() > PARK_TIMEOUT_SECONDS) {
            return true;
        }
        if (!parkFacingStart) {
            parkFacingStart = turnToward(startHeadingDeg);
            return false;
        }
        return driveToward(PARK_FORWARD_IN, PARK_LEFT_IN);
    }

    /**
     * Drives to the LOOK spot in two straight legs, forward then right, so the robot is off the
     * wall (and clear of the FLOWER beside it) before it moves sideways.
     * @return true at the LOOK spot, or after LOOK_TIMEOUT_SECONDS (AIM then starts from wherever it is).
     */
    boolean look() {
        if (stepTimer.seconds() > lookTimeoutSeconds()) {
            return true;
        }
        if (lookLeg == 0) {
            if (driveToward(LOOK_FORWARD_IN, 0)) {
                lookLeg = 1;
            }
            return false;
        }
        // right = minus "left"; the other LOOK spot is the same distance to the left.
        return driveToward(LOOK_FORWARD_IN, lookingOtherSide ? LOOK_RIGHT_IN : -LOOK_RIGHT_IN);
    }

    double lookTimeoutSeconds() {
        return lookingOtherSide ? OTHER_LOOK_TIMEOUT_SECONDS : LOOK_TIMEOUT_SECONDS;
    }

    /**
     * Once per Auto: if the camera works but no up CELL has been in view for
     * OTHER_SIDE_AFTER_SECONDS since LOOK, go to the other LOOK spot (see OTHER SIDE at the top).
     * @return true when it switched to LOOK, so the caller skips the rest of AIM or SEARCH.
     */
    boolean tryOtherSide() {
        if (lookingOtherSide || !vision.isEnabled() || vision.closestTag() != null
                || findTimer.seconds() < OTHER_SIDE_AFTER_SECONDS) {
            return false;
        }
        drive.stop();
        lookingOtherSide = true;
        lookLeg = 1;   // already forward: go straight across to the other side
        searchTargetDeg = startHeadingDeg + SEARCH_ANGLE_DEG;
        nextStep(AutonomousState.LOOK, "no up CELL here: trying other LOOK spot");
        return true;
    }

    /**
     * One loop of driving straight at a spot, measured from where the robot stood at INIT
     * (inches forward, inches to the left), while holding the start heading. Every loop it asks
     * odometry where the robot really is, so it does not matter how it got there.
     * @return true when the robot is within PARK_TOLERANCE_IN of the spot.
     */
    boolean driveToward(double forwardIn, double leftIn) {
        // How far the spot still is, in the INIT directions (x forward, y left)...
        double dx = forwardIn - drive.getPoseX();
        double dy = leftIn - drive.getPoseY();
        // Own-side guard: at MAX_FORWARD_IN, no more driving toward the middle of the FIELD.
        if (drive.getPoseX() >= MAX_FORWARD_IN && dx > 0) {
            dx = 0;
            if (!ownSideLimitLogged) {
                ownSideLimitLogged = true;
                logStep(autonomousState.name(), autonomousState.name(), "own-side limit (G402)");
            }
        }
        double distance = Math.hypot(dx, dy);
        if (distance < PARK_TOLERANCE_IN) {
            drive.stop();
            return true;
        }
        // ...turned into the robot's own directions, using the heading it has now.
        double h = Math.toRadians(drive.getPoseHeadingDeg());
        double ahead = dx * Math.cos(h) + dy * Math.sin(h);
        double toLeft = -dx * Math.sin(h) + dy * Math.cos(h);
        // Power grows with the distance left, between PARK_MIN_POWER and PARK_MAX_POWER.
        double power = Range.clip(PARK_GAIN * distance, PARK_MIN_POWER, PARK_MAX_POWER);
        double headingError = startHeadingDeg - drive.getPoseHeadingDeg();
        // In drive(), + strafe goes right and + rotate turns right, so "left" needs a minus sign.
        drive.drive(power * ahead / distance, -power * toLeft / distance, -PARK_HEADING_GAIN * headingError);
        return false;
    }

    /**
     * Turns in place at SEARCH_TURN_POWER toward a heading (degrees, + = left).
     * @return true when the robot is within HEADING_TOLERANCE_DEG of it.
     */
    boolean turnToward(double targetDeg) {
        double error = AngleUnit.normalizeDegrees(targetDeg - drive.getPoseHeadingDeg());
        if (Math.abs(error) < HEADING_TOLERANCE_DEG) {
            drive.stop();
            return true;
        }
        // + error means turn left, but in drive() a + rotate turns right, so we flip the sign.
        drive.drive(0, 0, error > 0 ? -SEARCH_TURN_POWER : SEARCH_TURN_POWER);
        return false;
    }
}
