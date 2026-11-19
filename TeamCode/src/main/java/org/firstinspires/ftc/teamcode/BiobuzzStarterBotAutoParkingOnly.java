package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BioBuzz StarterBot Auto Parking Only", group = "StarterBot")
public class BiobuzzStarterBotAutoParkingOnly extends OpMode {
    private MecanumDrive mecanumDrive = null;
    private boolean isComplete = false;
    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        mecanumDrive.init();
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        while(!isComplete) {
            mecanumDrive.driveDistance("FORWARD", 10, 2);
            isComplete = true;
        }
    }
    @Override
    public void stop() {
        
    }

}
