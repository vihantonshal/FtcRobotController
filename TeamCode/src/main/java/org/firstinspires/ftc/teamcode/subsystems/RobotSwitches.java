package org.firstinspires.ftc.teamcode.subsystems;

/*
 * FEATURE SWITCHES for the whole robot. Turn a part on (true) or off (false), then rebuild.
 * When a part is off, its subsystem does not look for its hardware and all its methods do
 * nothing, so the robot still drives while the build team has that part off the robot.
 * The TeleOp, the Auto and BioBuzzMotorMaxSpeed all read these, so they always agree.
 *
 *  USE_DRIVE     the four mecanum drive motors                (MecanumDriveSubsystem)
 *                false = the robot never moves (for testing only the camera / AprilTags)
 *  USE_INTAKE    intake motor + left/right intake servos        (IntakeSubsystem)
 *  USE_LAUNCHER  launcher motor + windmill servo                (LauncherSubsystem)
 *  USE_CAMERA    a camera and AprilTag aiming                   (AprilTagVision)
 *  USE_LIMELIGHT which camera, when USE_CAMERA is on: false = webcam "Webcam 1",
 *                true = Limelight 3A "limelight" (see AprilTagVision for its setup)
 *  USE_DASHBOARD FTC Dashboard: live graphs, camera picture and live number changes.
 *                Set to false for competition; telemetry then goes only to the Driver Station.
 *  USE_LAUNCH_ASSIST  TeleOp only: launch log, saving good launch poses (operator B) and
 *                auto-align to them (driver right bumper)                    (LaunchAssist)
 *  USE_FIELD_CENTRIC  TeleOp only: false = always robot-centric (stick up = the robot's front),
 *                and the driver's Y button does nothing. true = Y switches field-centric on/off
 *                (needs the "imu"), and FIELD_CENTRIC_AT_START picks the mode at INIT.
 */
public final class RobotSwitches {
    public static final boolean USE_DRIVE = true;
    public static final boolean USE_INTAKE = true;
    public static final boolean USE_LAUNCHER = true;
    public static final boolean USE_CAMERA = true;
    public static final boolean USE_LIMELIGHT = false;
    public static final boolean USE_DASHBOARD = true;
    public static final boolean USE_LAUNCH_ASSIST = true;
    public static final boolean USE_FIELD_CENTRIC = false;

    private RobotSwitches() {
    }
}
