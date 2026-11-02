package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Locale;

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
 * RUNWAY v10 (70 degree exit): USE_DISTANCE_SPEED is OFF and the launcher runs at one speed,
 * TARGET_VELOCITY x SPEED_SCALE, plus the operator's trim. The rest of this paragraph is for a 77 degree runway.
 * DISTANCE-BASED SPEED: a farther CELL needs a faster flywheel. Every loop the OpMode passes the
 * distance to the up CELL from the camera (webcam or Limelight) to setCellRange(). The target speed
 * then follows a straight line through two measured points, NEAR (NEAR_RANGE_IN, NEAR_VELOCITY) and
 * FAR (FAR_RANGE_IN, FAR_VELOCITY); closer than NEAR it stays at NEAR_VELOCITY, farther than FAR at
 * FAR_VELOCITY (no guessing past what was measured). With no CELL in view it keeps its last speed, so it
 * doesn't jump in the middle of a volley; before the first sighting it is TARGET_VELOCITY.
 * The range is the camera's AprilTag range: the straight line from the lens (CAMERA_HEIGHT_IN, 12 in)
 * up to the center of the CELL opening (about 59.5 in), so it is never below about 47.5 in. NEAR 54.4 in
 * is about 0.75 m along the floor and FAR 89.9 in about 2 m (from the release point; the front camera sits
 * 2 in ahead of it). The NEAR/FAR speeds are starting points from the 4-ball simulation
 * (Solidworks-MCP repo, DOCS/sim/hive_sim.py: Julia Runway v6, 77 degree exit toward the robot's FRONT
 * from 19.2 cm above the TILES, the CELL mouth's real lips, built on volley_sim.py,
 * launcher wheel + goBILDA 3628 flywheel, each ball's own speed drop included, aiming at the center of the
 * up CELL opening, 151 cm), not yet measured on the field: StarterBot_Change_Guide.md, "Distance-based flywheel speed". Shots are most
 * reliable from about 52-67 in of range (0.6-1.25 m along the floor): the up CELL's opening faces the robot
 * tipped 30 degrees up, so closer shots still rise too steeply to go in; 54.4 in (0.75 m) is the best entry
 * angle. The operator's trim (D-pad left/right
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
    // Runway v10 (70 degree exit): ONE speed for the whole shooting zone (see "Octave launcher check" below).
    // About 2526 RPM. With USE_DISTANCE_SPEED = true it is only used until the camera sees a CELL.
    public static double TARGET_VELOCITY = 1179;
    public static double FEED_MARGIN = 25;         // feed only above (target - this), e.g. 1225 for 1250
    public static double WINDMILL_POWER = 1.0;
    // The feed wheel (CR servo "feedWheel" beside the Runway lead-in) runs whenever the windmill does.
    // Seen from above it must turn clockwise so it pushes the ball toward the launcher: -1.0 if it turns the wrong way.
    public static double FEED_WHEEL_POWER = 1.0;
    // Distance-based speed (see the top of this file). Ranges in inches, speeds in ticks/s.
    // OFF for Runway v10: at 70 degrees the right speed hardly changes with distance. NEAR/FAR are the
    // Runway v6 (77 degree) line: set true again only with a 77 degree runway on the robot.
    public static boolean USE_DISTANCE_SPEED = false;
    public static double NEAR_RANGE_IN = 54.4;     // about 0.75 m along the floor (see the top of this file)
    public static double NEAR_VELOCITY = 1381;     // starting point from the simulation (v6 front launch), not measured yet
    public static double FAR_RANGE_IN = 89.9;      // about 2 m along the floor
    public static double FAR_VELOCITY = 1994;      // starting point from the simulation (v6 front launch), not measured yet
    public static double TRIM_STEP = 25;           // ticks/s per operator D-pad press
    /*
     * Octave launcher check (2026-10-26, v4: 96 mm Hogback drives the ball, Runway v10 70 degree exit).
     * TARGET_VELOCITY 1179 scored all 4 POLLEN from the robot front about 12-32 in from the HIVE stand
     * (tape measure from the intake wheels; best spot about 15 in), window 1148-1210 (+/-2.7%).
     * SPEED_SCALE multiplies TARGET_VELOCITY and every distance-based speed. 1179 assumes the ball leaves
     * at 0.45 x the Hogback tread speed: film one shot at 1179 (the model says 18.75 ft/s) and set
     * SPEED_SCALE = 18.75 / measured ft/s (1.2 if it leaves at 0.375 x, the older estimate).
     * ZONE_MIN/MAX_RANGE_IN: that zone in the SDK's ftcPose.range (hypot(x, y): it leaves out the
     * camera's z), about 51.0-66.7 in with the camera tilted 38 degrees, 52.9-67.0 in at 45.
     * FEED_MAX_OVER: also don't feed above (target + this), e.g. right after a spin-up overshoot,
     * because a too-fast ball flies long. 30 = the one-speed window; FEED_MARGIN is the low side.
     */
    public static double SPEED_SCALE = 1.0;
    public static double ZONE_MIN_RANGE_IN = 51;
    public static double ZONE_MAX_RANGE_IN = 67;
    public static double FEED_MAX_OVER = 30;
    /*
     * SMOOTH MODE (new LauncherSubsystem(hardwareMap, true), used by the tank TeleOp): our own speed
     * control instead of the hub's PIDF, for a flywheel that jerks with the hub's control.
     *   power = SMOOTH_KV * target + SMOOTH_KP * (target - speed), never below 0
     * Never below 0: when the wheel is too fast it coasts down instead of being braked backward (the
     * hub's PIDF brakes on every overshoot, which jerks a light flywheel). The speed reading is smoothed
     * with SMOOTH_FILTER (0..1, lower = smoother but slower to react) against encoder noise.
     * SMOOTH_KV: power per tick/s; 1/2800 = full power at about 2800 ticks/s (6000 RPM, 28 ticks/turn).
     * If the speed stays near 0 for ENCODER_FAULT_SECONDS while powered, the encoder isn't counting:
     * it stops correcting (no runaway at full power), feeds after SPINUP_SECONDS, and warns.
     * PIDF above is not used in smooth mode.
     */
    public static double SMOOTH_KV = 1.0 / 2800;
    public static double SMOOTH_KP = 0.0005;
    public static double SMOOTH_FILTER = 0.3;
    public static double ENCODER_FAULT_SECONDS = 1.0;
    // With the goBILDA 3628-0032-0082 steel flywheel (152 g, 1651 g*cm^2) on the 96 mm Hogback wheel,
    // spin-up takes about 0.8 s to ~3000 RPM and about 1.2 s to the far-shot speed, so 1.5 s still covers it.
    public static double SPINUP_SECONDS = 1.5;
    /*
     * PULSE FEEDING (setPulseFeed(true), used by the tank TeleOp): while shooting is held, the windmill
     * feeds in turns so it keeps launching. First it waits for the launcher to reach speed, then:
     * feed FEED_ON_SECONDS, pause at least FEED_OFF_SECONDS so the flywheel recovers, then feed again
     * once it is back at speed, or after RECOVER_MAX_SECONDS at most (so it never stalls).
     */
    public static double FEED_ON_SECONDS = 0.3;
    public static double FEED_OFF_SECONDS = 0.3;
    public static double RECOVER_MAX_SECONDS = 1.0;

    private final boolean enabled = RobotSwitches.USE_LAUNCHER;
    private DcMotorEx launcher;
    //private CRServo windmill;
    private CRServo feedWheel;   // null when the configuration has no "feedWheel"
    private DcMotorEx roller;

    private boolean feeding = false;
    // The PIDF numbers the motor has now (NaN = never sent), and the speed it was last asked for.
    private final PIDFCoefficients sentPidf = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    private double targetVelocity = 0;
    // The distance-based speed (NaN until the camera has seen a CELL), the last CELL distance, the trim.
    private double distanceSpeed = Double.NaN;
    private double cellRangeIn = Double.NaN;
    private double trim = 0;
    // Smooth mode: the smoothed speed, the power sent, the encoder check, and time since spin-up began.
    private final boolean smooth;
    private double smoothVelocity = 0;
    private double smoothPower = 0;
    private boolean encoderFault = false;
    private final ElapsedTime slowTimer = new ElapsedTime();
    private final ElapsedTime spinTimer = new ElapsedTime();
    // Pulse feeding: on or off, the step it is in, and how long it has been in that step.
    private enum FeedStep { SPIN_UP, FEED, RECOVER }
    private boolean pulseFeed = false;
    private FeedStep feedStep = FeedStep.SPIN_UP;
    private final ElapsedTime feedTimer = new ElapsedTime();
    // TeleOp: launcher on or off (the operator's button), and whether it has reached speed since then.
    private boolean on = false;
    private boolean wasReady = false;
    private boolean justReady = false;

    // The hub's PIDF speed control (mecanum TeleOp and Auto).
    public LauncherSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    // smooth = true: our own gentler speed control (see SMOOTH MODE above).
    public LauncherSubsystem(HardwareMap hardwareMap, boolean smooth) {
        this.smooth = smooth;
        if (!enabled) {
            return;
        }
        launcher = hardwareMap.tryGet(DcMotorEx.class, "launcher");
        if (launcher == null) {
            throw new IllegalArgumentException("No motor named \"launcher\" in the ACTIVE configuration. Its motors are: "
                    + hardwareMap.getAllNames(DcMotor.class) + " (rename one to launcher, or set RobotSwitches.USE_LAUNCHER = false)");
        }
        // The mecanum robot's configuration calls it "windmillServo", the tank robot's "windmill".
        // Neither in the configuration = no windmill: the launcher still spins, feeding moves nothing.
        CRServo found = hardwareMap.tryGet(CRServo.class, "windmillServo");
        //windmill = found != null ? found : hardwareMap.tryGet(CRServo.class, "windmill");
        roller = hardwareMap.get(DcMotorEx.class, "roller");
        roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedWheel = hardwareMap.tryGet(CRServo.class, "feedWheel");


        if (smooth) {
            // We set the power ourselves; the encoder still counts and getVelocity() still works.
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast, never brake
        } else {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // The Control Hub runs the PIDF     loop itself; we only give it the numbers.
            sendPidfIfChanged();
        }

        setFeedPower(false);
        if (roller != null) {

            //windmill.setDirection(DcMotorSimple.Direction.REVERSE);
            roller.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }

    // The windmill and the feed wheel feed together.
    private void setFeedPower(boolean feed) {
        //if (windmill != null) {
          //  windmill.setPower(feed ? WINDMILL_POWER : 0);
        roller.setPower(feed ? 1 : 0);


        if (feedWheel != null) {
            feedWheel.setPower(feed ? FEED_WHEEL_POWER : 0);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    // Which way the windmill turns (REVERSE by default). For a robot whose windmill is mounted the other way.
    public void setWindmillDirection(DcMotorSimple.Direction direction) {
        if (enabled && roller  != null) {

            //windmill.setDirection(direction);
            roller.setDirection(direction);

        }
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

    // The same, from the camera's up CELL (null = none in view).
    public void setCell(AprilTagClusterDetection cell) {
        setCellRange(cell != null ? cell.ftcPose.range : Double.NaN);
    }

    // ---------------- TeleOp: on/off and feeding ----------------

    // The operator's on/off button. While on, the launcher keeps spinning.
    public void toggleOn() {
        if (enabled) {
            on = !on;
        }
    }

    public boolean isOn() {
        return on;
    }

    /*
     * TeleOp, once per loop. Off: stopped. On: spinning, and feeding while feedButton is held (the
     * windmill waits until the launcher is fast enough).
     */
    public void run(boolean feedButton) {
        if (!on) {
            stop();
            wasReady = false;
        } else if (feedButton) {
            update(true);
        } else {
            spinUp();
        }
        // Ready once per turn-on: wasReady stays true until the launcher is turned off, so the small
        // speed dip after each shot doesn't count again.
        boolean ready = on && isAtSpeed();
        justReady = ready && !wasReady;
        if (justReady) {
            wasReady = true;
        }
    }

    // True in the one loop where the launcher first reaches speed after being turned on: rumble now.
    public boolean justBecameReady() {
        return justReady;
    }

    // "off", "spinning up", "ready" or "feeding", for telemetry.
    public String getStateText() {
        if (!on) {
            return "off";
        }
        return feeding ? "feeding" : (isAtSpeed() ? "ready" : "spinning up");
    }

    // The launcher lines the drive team always sees: flywheel speed and direction, and the shot speed.
    public void addDriverLines(Telemetry telemetry) {
        telemetry.addData("Flywheel", "%s %.0f of %.0f ticks/s (%.0f RPM)",
                getDirection(), getVelocity(), getShootTarget(), getRpm());
        telemetry.addData("Shot speed", "%s, trim %+.0f (operator D-pad left/right)",
                Double.isNaN(cellRangeIn) ? "no CELL seen yet" : String.format(Locale.US, "for a CELL %.0f in away, %s",
                        cellRangeIn, isInShootingZone() ? "in zone"
                                : String.format(Locale.US, "OUTSIDE the %.0f-%.0f in zone", ZONE_MIN_RANGE_IN, ZONE_MAX_RANGE_IN)),
                trim);
        if (on && "REVERSE".equals(getDirection())) {
            telemetry.addLine("WARNING: the flywheel spins BACKWARD");
        }
        telemetry.addData("Roller is On", roller.getPower());
        telemetry.addData("Roller velocity", roller.getVelocity());

    }

    // The flywheel speed for a CELL this far away: the straight line through NEAR and FAR, held at the
    // ends, times SPEED_SCALE.
    public static double speedForRange(double rangeIn) {
        if (FAR_RANGE_IN <= NEAR_RANGE_IN) {
            return NEAR_VELOCITY * SPEED_SCALE;
        }
        double t = Range.clip((rangeIn - NEAR_RANGE_IN) / (FAR_RANGE_IN - NEAR_RANGE_IN), 0, 1);
        return (NEAR_VELOCITY + t * (FAR_VELOCITY - NEAR_VELOCITY)) * SPEED_SCALE;
    }

    // True when the last CELL distance is inside the simulated shooting zone (ZONE_MIN/MAX_RANGE_IN).
    public boolean isInShootingZone() {
        return cellRangeIn >= ZONE_MIN_RANGE_IN && cellRangeIn <= ZONE_MAX_RANGE_IN;   // false for NaN
    }

    // The operator's practice trim: + or - TRIM_STEP per press, added to the shooting speed.
    public void adjustTrim(int presses) {
        trim += presses * TRIM_STEP;
    }

    // The speed the launcher spins at when on: distance-based (or TARGET_VELOCITY) plus the trim.
    public double getShootTarget() {
        double base = USE_DISTANCE_SPEED && !Double.isNaN(distanceSpeed) ? distanceSpeed : TARGET_VELOCITY * SPEED_SCALE;
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
        setTarget(shoot ? getShootTarget() : 0);
        feeding = pulseFeed ? pulseFeeding(shoot) : shoot && isAtSpeed();
        setFeedPower(feeding);
    }

    // Spin up without feeding, so the launcher is already fast when it is time to shoot.
    public void spinUp() {
        if (!enabled) {
            return;
        }
        setTarget(getShootTarget());
        feeding = false;
        setFeedPower(false);
    }

    public void stop() {
        update(false);
    }

    // Runs the launcher toward this speed (0 = stop): the hub's PIDF, or our smooth control.
    private void setTarget(double target) {
        if (smooth && targetVelocity <= 0 && target > 0) {
            spinTimer.reset();   // spin-up starts now
        }
        targetVelocity = target;
        if (!smooth) {
            sendPidfIfChanged();
            launcher.setVelocity(target);
            return;
        }
        smoothVelocity += SMOOTH_FILTER * (launcher.getVelocity() - smoothVelocity);
        if (target <= 0) {
            smoothPower = 0;
            slowTimer.reset();
        } else {
            double correction = encoderFault ? 0 : SMOOTH_KP * (target - smoothVelocity);
            smoothPower = Range.clip(SMOOTH_KV * target + correction, 0, 1);
            // Powered but the encoder still reads about 0: it isn't counting (cable, port or direction).
            if (smoothVelocity > 100) {
                slowTimer.reset();
            } else if (slowTimer.seconds() > ENCODER_FAULT_SECONDS) {
                encoderFault = true;
            }
        }
        launcher.setPower(smoothPower);
    }

    // true = feed in timed pulses while shooting is held (see PULSE FEEDING above).
    public void setPulseFeed(boolean on) {
        pulseFeed = on;
    }

    // One step of pulse feeding; returns true while the windmill should feed.
    private boolean pulseFeeding(boolean shoot) {
        if (!shoot) {
            feedStep = FeedStep.SPIN_UP;
            return false;
        }
        double t = feedTimer.seconds();
        switch (feedStep) {
            case SPIN_UP:
                if (isAtSpeed()) {
                    startFeedStep(FeedStep.FEED);
                }
                break;
            case FEED:
                if (t >= FEED_ON_SECONDS) {
                    startFeedStep(FeedStep.RECOVER);
                }
                break;
            case RECOVER:
                if (t >= FEED_OFF_SECONDS && (isAtSpeed() || t >= RECOVER_MAX_SECONDS)) {
                    startFeedStep(FeedStep.FEED);
                }
                break;
        }
        return feedStep == FeedStep.FEED;
    }

    private void startFeedStep(FeedStep step) {
        feedStep = step;
        feedTimer.reset();
    }

    // A problem to show the driver, or null: the encoder check in smooth mode.
    public String getProblem() {
        return encoderFault ? "ENCODER NOT COUNTING: check the launcher encoder cable and port. Running on fixed power" : null;
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

    // True when the launcher is at the right speed to feed: above (target - FEED_MARGIN) and below
    // (target + FEED_MAX_OVER).
    public boolean isAtSpeed() {
        if (!enabled) {
            return false;
        }
        if (smooth) {
            return encoderFault ? spinTimer.seconds() > SPINUP_SECONDS : inFeedWindow(smoothVelocity);
        }
        return inFeedWindow(launcher.getVelocity());
    }

    private boolean inFeedWindow(double velocity) {
        double target = getShootTarget();
        return velocity > target - FEED_MARGIN && velocity < target + FEED_MAX_OVER;
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
            if (smooth) {
                telemetry.addData("launcher smoothed velocity", smoothVelocity);
                telemetry.addData("launcher power", smoothPower);
            }
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
