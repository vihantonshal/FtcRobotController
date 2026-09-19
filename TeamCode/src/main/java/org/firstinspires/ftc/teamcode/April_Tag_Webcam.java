package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import Mechanisms.AprilTagWebcam;
@Autonomous
public class April_Tag_Webcam extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();




    @Override
    public void init() {
        aprilTagWebcam.init( hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }
}
