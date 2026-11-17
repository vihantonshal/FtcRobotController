package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSwitches;

/*
 * PARKING ONLY Auto: the robot drives FORWARD_IN (25) inches straight forward, then stops.
 * It does not launch, does not use the camera and does not need the alliance picked.
 *
 * How to use: place the robot pointing the way it must go, press INIT, then START.
 * It uses only the drive (MecanumDriveSubsystem), the same odometry go-to as BioBuzzStarterBotAuto.
 */
@Config
@Autonomous(name = "BioBuzz StarterBot Auto Parking Only", group = "StarterBot")
public class BioBuzzStarterBotAutoParkingOnly extends OpMode {

    // How far to drive forward, in inches. Can be changed in FTC Dashboard before INIT.
    public static double FORWARD_IN = 22;
    // Safety: stop after this many seconds even if the robot has not reached the spot (e.g. it is blocked).
    public static double TIMEOUT_SECONDS = 8;

    private MecanumDriveSubsystem drive;
    private final ElapsedTime timer = new ElapsedTime();

    // Where the robot was when START was pressed: the move is measured from here.
    private double startForwardIn;
    private double startLeftIn;
    private double startHeadingDeg;
    private boolean done = false;
    private String result = "driving";

    @Override
    public void init() {
        if (RobotSwitches.USE_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        // Bulk reads (SDK): each hub sends all its motor encoders in one message per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drive = new MecanumDriveSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Plan", "drive %.0f in forward, then stop", FORWARD_IN);
    }

    @Override
    public void start() {
        timer.reset();
        drive.update();
        startForwardIn = drive.getPoseX();
        startLeftIn = drive.getPoseY();
        startHeadingDeg = drive.getPoseHeadingDeg();
    }

    @Override
    public void loop() {
        drive.update();   // where the robot is (odometry)

        if (!done) {
            double targetForwardIn = startForwardIn + FORWARD_IN;
            if (drive.driveToPoint(targetForwardIn, startLeftIn, startHeadingDeg, targetForwardIn)) {
                done = true;
                result = "PARKED";
            } else if (timer.seconds() > TIMEOUT_SECONDS) {
                done = true;
                result = "STOPPED by the timeout";
            }
        }
        if (done) {
            drive.stop();
        }

        telemetry.addData("Parking", result);
        telemetry.addData("Driven", "%.1f of %.0f in forward", drive.getPoseX() - startForwardIn, FORWARD_IN);
        telemetry.addData("Time", "%.1f s", timer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
