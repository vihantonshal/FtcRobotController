package org.firstinspires.ftc.teamcode.subsystems;

/*
 * FEATURE SWITCHES for the whole robot. Turn a part on (true) or off (false), then rebuild.
 * When a part is off, its subsystem does not look for its hardware and all its methods do
 * nothing, so the robot still drives while the build team has that part off the robot.
 * The TeleOp, the Auto and BioBuzzMotorMaxSpeed all read these, so they always agree.
 *
 *  USE_INTAKE    intake motor + left/right intake servos        (IntakeSubsystem)
 *  USE_LAUNCHER  launcher motor + windmill servo                (LauncherSubsystem)
 *  USE_CAMERA    a camera and AprilTag aiming                   (AprilTagVision)
 *  USE_LIMELIGHT which camera, when USE_CAMERA is on: false = webcam "Webcam 1",
 *                true = Limelight 3A "limelight" (see AprilTagVision for its setup)
 *  USE_DASHBOARD FTC Dashboard: live graphs, camera picture and live number changes.
 *                Set to false for competition; telemetry then goes only to the Driver Station.
 */
public final class RobotSwitches {
    public static final boolean USE_INTAKE = true;
    public static final boolean USE_LAUNCHER = true;
    public static final boolean USE_CAMERA = true;
    public static final boolean USE_LIMELIGHT = false;
    public static final boolean USE_DASHBOARD = true;

    private RobotSwitches() {
    }
}
