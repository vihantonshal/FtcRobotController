package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * The launcher (5203-2402-0019 converted to 1:1, about 6000 RPM, 96 mm wheel) and the windmill
 * servo that feeds balls into it. Switched on and off with RobotSwitches.USE_LAUNCHER; when it is
 * off, nothing here looks for hardware and every method does nothing.
 *
 * Speeds are in encoder ticks per second. This motor counts 28 ticks per turn, so
 * RPM = ticks per second / 28 * 60: 1250 ticks/s is about 2679 RPM.
 * The windmill only feeds once the launcher is within FEED_MARGIN of its target speed, so every ball
 * gets a good throw.
 *
 * DISTANCE-BASED SPEED: a farther CELL needs a faster flywheel. Every loop the OpMode passes the
 * distance to the up CELL from the camera (webcam or Limelight) to setCellRange(). The target speed
 * then follows a straight line through two measured points, NEAR (NEAR_RANGE_IN, NEAR_VELOCITY) and
 * FAR (FAR_RANGE_IN, FAR_VELOCITY); closer than NEAR it stays at NEAR_VELOCITY, farther than FAR at
 * FAR_VELOCITY (no guessing past what was measured). With no CELL in view it keeps its last speed, so it
 * doesn't jump in the middle of a volley; before the first sighting it is TARGET_VELOCITY.
 * Both points start at TARGET_VELOCITY, so nothing changes until the launcher team measures them:
 * StarterBot_Change_Guide.md, "Distance-based flywheel speed". The operator's trim (D-pad left/right
 * in the TeleOp) adds to the speed while practicing, to find what scores at each distance. Tune PIDF in FTC Dashboard under "LauncherSubsystem" while the launcher spins: a changed
 * number reaches the motor within one loop. Graph "launcher velocity" against "launcher target". When it
 * looks good, copy the numbers into PIDF below, because Dashboard changes are lost when the robot restarts.
 *
 * If the speed jumps far above the target and cannot be controlled, check that the encoder cable
 * is in the Encoder port with the same number as the launcher's Motor port.
 */
@Config
public class LauncherSubsystem {
    // ---- Tuning numbers (owner: launcher team) ----
    public static PIDFCoefficients PIDF = new PIDFCoefficients(40, 0, 0, 12.5);   // p, i, d, f
    public static double TARGET_VELOCITY = 1250;   // about 2679 RPM; used until the camera sees a CELL
    public static double FEED_MARGIN = 50;         // feed only above (target - this), e.g. 1200 for 1250
    public static double WINDMILL_POWER = 1.0;
    // Distance-based speed (see the top of this file). Ranges in inches, speeds in ticks/s.
    public static boolean USE_DISTANCE_SPEED = true;
    public static double NEAR_RANGE_IN = 30;
    public static double NEAR_VELOCITY = 1250;
    public static double FAR_RANGE_IN = 60;
    public static double FAR_VELOCITY = 1250;
    public static double TRIM_STEP = 25;           // ticks/s per operator D-pad press

    private final boolean enabled = RobotSwitches.USE_LAUNCHER;
    private DcMotorEx launcher;
    private CRServo windmill;
    private boolean feeding = false;
    // The PIDF numbers the motor has now (NaN = never sent), and the speed it was last asked for.
    private final PIDFCoefficients sentPidf = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    private double targetVelocity = 0;
    // The distance-based speed (NaN until the camera has seen a CELL), the last CELL distance, the trim.
    private double distanceSpeed = Double.NaN;
    private double cellRangeIn = Double.NaN;
    private double trim = 0;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        if (!enabled) {
            return;
        }
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(CRServo.class, "windmillServo");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The Control Hub runs the PIDF loop itself; we only give it the numbers.
        sendPidfIfChanged();

        windmill.setPower(0);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /*
     * Call once per loop, before update() or spinUp(): the distance to the up CELL in inches, from the
     * camera, or Double.NaN when no CELL is in view (the speed then stays where it was).
     */
    public void setCellRange(double rangeIn) {
        if (Double.isNaN(rangeIn)) {
            return;
        }
        cellRangeIn = rangeIn;
        distanceSpeed = speedForRange(rangeIn);
    }

    // The flywheel speed for a CELL this far away: the straight line through NEAR and FAR, held at the ends.
    public static double speedForRange(double rangeIn) {
        if (FAR_RANGE_IN <= NEAR_RANGE_IN) {
            return NEAR_VELOCITY;
        }
        double t = Range.clip((rangeIn - NEAR_RANGE_IN) / (FAR_RANGE_IN - NEAR_RANGE_IN), 0, 1);
        return NEAR_VELOCITY + t * (FAR_VELOCITY - NEAR_VELOCITY);
    }

    // The operator's practice trim: + or - TRIM_STEP per press, added to the shooting speed.
    public void adjustTrim(int presses) {
        trim += presses * TRIM_STEP;
    }

    // The speed the launcher spins at when on: distance-based (or TARGET_VELOCITY) plus the trim.
    public double getShootTarget() {
        double base = USE_DISTANCE_SPEED && !Double.isNaN(distanceSpeed) ? distanceSpeed : TARGET_VELOCITY;
        return base + trim;
    }

    // The last CELL distance passed to setCellRange(), in inches (NaN if none yet), and the trim.
    public double getCellRange() {
        return cellRangeIn;
    }

    public double getTrim() {
        return trim;
    }

    /*
     * Call once per loop. shoot = true spins the launcher up to getShootTarget() and feeds with the
     * windmill once it is fast enough; shoot = false stops both.
     */
    public void update(boolean shoot) {
        if (!enabled) {
            return;
        }
        sendPidfIfChanged();
        targetVelocity = shoot ? getShootTarget() : 0;
        launcher.setVelocity(targetVelocity);
        feeding = shoot && isAtSpeed();
        windmill.setPower(feeding ? WINDMILL_POWER : 0);
    }

    // Spin up without feeding, so the launcher is already fast when it is time to shoot.
    public void spinUp() {
        if (!enabled) {
            return;
        }
        sendPidfIfChanged();
        targetVelocity = getShootTarget();
        launcher.setVelocity(targetVelocity);
        feeding = false;
        windmill.setPower(0);
    }

    public void stop() {
        update(false);
    }

    // Launcher speed in ticks per second: + = spinning the launch way, - = backward. 0 when switched off.
    public double getVelocity() {
        return enabled ? launcher.getVelocity() : 0;
    }

    // Launcher speed in RPM (28 ticks per turn on the 1:1 motor).
    public double getRpm() {
        return getVelocity() / 28.0 * 60.0;
    }

    /*
     * Which way the flywheel spins: FORWARD (launches balls), REVERSE (wrong way: check the motor
     * direction or the wiring), STOPPED, or OFF when USE_LAUNCHER is false.
     */
    public String getDirection() {
        if (!enabled) {
            return "OFF";
        }
        double v = launcher.getVelocity();
        if (Math.abs(v) < 20) {
            return "STOPPED";
        }
        return v > 0 ? "FORWARD" : "REVERSE";
    }

    // True when the launcher is fast enough to feed: within FEED_MARGIN of the shooting speed.
    public boolean isAtSpeed() {
        return enabled && launcher.getVelocity() > getShootTarget() - FEED_MARGIN;
    }

    // True while the windmill is feeding balls. The intake helps push them along.
    public boolean isFeeding() {
        return feeding;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (enabled) {
            telemetry.addData("launcher velocity", launcher.getVelocity());
            telemetry.addData("launcher target", targetVelocity);
            telemetry.addData("launcher CELL range", cellRangeIn);
        }
    }

    // Sends PIDF to the motor when a number changed (in FTC Dashboard), not every loop.
    private void sendPidfIfChanged() {
        if (PIDF.p == sentPidf.p && PIDF.i == sentPidf.i && PIDF.d == sentPidf.d && PIDF.f == sentPidf.f) {
            return;
        }
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        sentPidf.p = PIDF.p;
        sentPidf.i = PIDF.i;
        sentPidf.d = PIDF.d;
        sentPidf.f = PIDF.f;
    }
}
