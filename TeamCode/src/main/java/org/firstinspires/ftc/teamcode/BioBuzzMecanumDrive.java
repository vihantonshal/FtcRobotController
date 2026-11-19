package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Optimized Mecanum drivetrain helper for BioBuzzStarterBotAuto and TeleOp.
 */
public class BioBuzzMecanumDrive {

    // Rev Control Hub Orientation (Hub upright on RIGHT side, USB pointing BACKWARD)
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // Physical robot constants (GoBILDA 312 RPM motors + 104mm Mecanum wheels)
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4; // ~4.094 in
    public static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Drivetrain tuning
    public static final double STRAFE_CORRECTION = 1.15; // Mecanum sideways slip multiplier
    public static final double START_POWER = 0.20;
    public static final double RAMP_UP_PER_INCH = 0.10;
    public static final double MIN_POWER = 0.15;
    public static final double SLOW_DOWN_PER_INCH = 0.08;
    public static final double HEADING_GAIN = 0.025;      // Heading stabilization gain
    public static final double MAX_CORRECTION = 0.30;
    public static final double CROSS_GAIN = 0.05;         // Strafe drift compensation
    public static final double MAX_CROSS = 0.20;

    // Motor hardware names
    public static final String FL_NAME = "front_left_motor";
    public static final String FR_NAME = "front_right_motor";
    public static final String BL_NAME = "back_left_motor";
    public static final String BR_NAME = "back_right_motor";

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    private DcMotor[] motors;

    public IMU imu = null;
    private double holdHeading = 0.0;
    public volatile boolean stopRequested = false;

    public BioBuzzMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        stopRequested = false;

        frontLeftDrive = hardwareMap.get(DcMotor.class, FL_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FR_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BL_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BR_NAME);

        motors = new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};

        // Left motors reversed for forward motion
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(BRAKE);
        }

        resetEncoders();

        // Safely initialize IMU
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            try {
                imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
                imu.resetYaw();
            } catch (Exception e) {
                if (telemetry != null) {
                    telemetry.addData("IMU Init Warning", e.getMessage());
                }
            }
        }
        holdHeading = getHeadingDeg();
    }

    public void resetEncoders() {
        for (DcMotor motor : motors) {
            if (motor != null) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void stop() {
        stopRequested = true;
        setPower(0.0);
        if (motors != null) {
            for (DcMotor motor : motors) {
                if (motor != null) {
                    motor.setZeroPowerBehavior(BRAKE);
                }
            }
        }
    }

    public double getHeadingDeg() {
        if (imu != null) {
            try {
                return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            } catch (Exception ignored) {
            }
        }
        return 0.0;
    }

    public void resetHeading() {
        if (imu != null) {
            try {
                imu.resetYaw();
            } catch (Exception ignored) {
            }
        }
        holdHeading = 0.0;
    }

    public void setHoldHeading(double degrees) {
        holdHeading = degrees;
    }

    /**
     * Drives a distance with explicit DistanceUnit (e.g. DistanceUnit.CM or DistanceUnit.INCH).
     */
    public void driveDistance(String direction, double distance, DistanceUnit unit, double speed) {
        double inches = unit.toInches(Math.abs(distance));
        driveDistance(direction, inches, speed);
    }

    /**
     * Drives distance in inches along direction with heading hold.
     * @param direction "FORWARD", "BACK", "LEFT", or "RIGHT"
     * @param inches    Distance in inches
     * @param speed     Speed 0.0 to 1.0
     */
    public void driveDistance(String direction, double inches, double speed) {
        if (stopRequested || Thread.currentThread().isInterrupted()) return;

        double fwdDir = 0, sideDir = 0;
        if (direction.equalsIgnoreCase("FORWARD")) {
            fwdDir = 1;
        } else if (direction.equalsIgnoreCase("BACK")) {
            fwdDir = -1;
        } else if (direction.equalsIgnoreCase("RIGHT")) {
            sideDir = 1;
        } else if (direction.equalsIgnoreCase("LEFT")) {
            sideDir = -1;
        } else {
            if (telemetry != null) {
                telemetry.addData("Error", "Unknown direction: %s", direction);
                telemetry.update();
            }
            return;
        }

        boolean strafing = (sideDir != 0);
        double ticksPerInch = COUNTS_PER_INCH * (strafing ? STRAFE_CORRECTION : 1.0);
        double targetTicks = Math.abs(inches) * ticksPerInch;

        int flStart = frontLeftDrive.getCurrentPosition();
        int frStart = frontRightDrive.getCurrentPosition();
        int blStart = backLeftDrive.getCurrentPosition();
        int brStart = backRightDrive.getCurrentPosition();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double safeSpeed = Math.max(0.1, Math.min(1.0, Math.abs(speed)));
        double expectedSeconds = (Math.abs(inches) / (safeSpeed * 24.0));
        double timeout = 1.5 + expectedSeconds * 1.5;

        ElapsedTime timer = new ElapsedTime();
        double aim = holdHeading;

        while (!stopRequested && !Thread.currentThread().isInterrupted() && timer.seconds() < timeout) {
            int dFL = frontLeftDrive.getCurrentPosition() - flStart;
            int dFR = frontRightDrive.getCurrentPosition() - frStart;
            int dBL = backLeftDrive.getCurrentPosition() - blStart;
            int dBR = backRightDrive.getCurrentPosition() - brStart;

            double fwdTicks = (dFL + dFR + dBL + dBR) / 4.0;
            double sideTicks = (dFL - dFR - dBL + dBR) / 4.0;
            double traveledTicks = fwdDir * fwdTicks + sideDir * sideTicks;

            if (traveledTicks >= targetTicks) break;

            double traveledIn = traveledTicks / ticksPerInch;
            double remainingIn = Math.abs(inches) - traveledIn;

            // Trapezoidal velocity profile
            double p = Math.min(safeSpeed, START_POWER + traveledIn * RAMP_UP_PER_INCH);
            p = Math.min(p, MIN_POWER + remainingIn * SLOW_DOWN_PER_INCH);

            // Gyro heading correction: steers robot back to aim heading
            double error = AngleUnit.normalizeDegrees(aim - getHeadingDeg());
            double turn = Range.clip(-HEADING_GAIN * error, -MAX_CORRECTION, MAX_CORRECTION);

            // Cross-track drift correction
            double driftIn = strafing ? fwdTicks / COUNTS_PER_INCH
                    : sideTicks / (COUNTS_PER_INCH * STRAFE_CORRECTION);
            double c = Range.clip(-CROSS_GAIN * driftIn, -MAX_CROSS, MAX_CROSS);

            double f = fwdDir * p + (strafing ? c : 0);
            double s = sideDir * p + (strafing ? 0 : c);

            setMecanum(f, s, turn);

            if (telemetry != null) {
                telemetry.addData("Drive", "%s %.1fin @ %.2f", direction, inches, p);
                telemetry.addData("Heading Err", "%.1f°", error);
                telemetry.update();
            }
        }

        setPower(0.0);
    }

    public void turnToHeading() {
        turnToHeading(holdHeading);
    }

    public void turnToHeading(double targetDeg) {
        if (stopRequested || Thread.currentThread().isInterrupted()) return;

        ElapsedTime timer = new ElapsedTime();
        holdHeading = targetDeg;

        while (!stopRequested && !Thread.currentThread().isInterrupted() && timer.seconds() < 1.5) {
            double err = AngleUnit.normalizeDegrees(targetDeg - getHeadingDeg());
            if (Math.abs(err) < 1.0) break; // 1 degree tolerance

            double turn = Range.clip(-0.02 * err, -0.35, 0.35);
            if (Math.abs(turn) < 0.09) {
                turn = Math.copySign(0.09, turn);
            }

            frontLeftDrive.setPower(turn);
            backLeftDrive.setPower(turn);
            frontRightDrive.setPower(-turn);
            backRightDrive.setPower(-turn);
        }
        setPower(0.0);
    }

    public void turnDegrees(double degrees, double speed) {
        turnToHeading(AngleUnit.normalizeDegrees(getHeadingDeg() + degrees));
    }

    public void mecanumDrive(double forward, double strafe, double rotate) {
        final double MAX_SPEED = 0.85;
        final double EXPONENT = 3.0;

        double f = Math.copySign(Math.pow(Math.abs(forward), EXPONENT), forward);
        double s = Math.copySign(Math.pow(Math.abs(strafe), EXPONENT), strafe);
        double r = Math.copySign(Math.pow(Math.abs(rotate), EXPONENT), rotate);

        double fl = f + s + r;
        double fr = f - s - r;
        double bl = f - s + r;
        double br = f + s - r;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeftDrive.setPower((fl / max) * MAX_SPEED);
        frontRightDrive.setPower((fr / max) * MAX_SPEED);
        backLeftDrive.setPower((bl / max) * MAX_SPEED);
        backRightDrive.setPower((br / max) * MAX_SPEED);
    }

    private void setMecanum(double f, double s, double r) {
        double fl = f + s + r;
        double fr = f - s - r;
        double bl = f - s + r;
        double br = f + s - r;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    private void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) {
            if (motor != null) motor.setMode(mode);
        }
    }

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            if (motor != null) motor.setPower(power);
        }
    }
}