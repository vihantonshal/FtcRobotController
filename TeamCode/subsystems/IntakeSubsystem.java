package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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
 * The intake: one roller motor (5203-2402-0051, 117 RPM) and two continuous-rotation servos that
 * pull balls out of corners. Switched on and off with RobotSwitches.USE_INTAKE; when it is off,
 * nothing here looks for hardware and run() does nothing.
 *
 * The roller uses velocity control: a power from -1 to 1 becomes a fraction of MAX_TICKS_PER_SEC.
 * Tune PIDF in FTC Dashboard under "IntakeSubsystem" while the robot runs: a changed number reaches
 * the motor within one loop. Graph "intake velocity" against "intake target". When it looks good,
 * copy the numbers into PIDF below, because Dashboard changes are lost when the robot restarts.
 */
@Config
public class IntakeSubsystem {
    // ---- Tuning numbers (owner: intake team) ----
    // Starting values from the FTC guide; measure the real top speed with BioBuzzMotorMaxSpeed.
    public static PIDFCoefficients PIDF = new PIDFCoefficients(1.17, 0.117, 0, 11.7);   // p, i, d, f
    public static double MAX_TICKS_PER_SEC = 2500;

    private final boolean enabled = RobotSwitches.USE_INTAKE;
    private DcMotorEx intake;
    private CRServo leftServo;
    private CRServo rightServo;
    // The PIDF numbers the motor has now (NaN = never sent), and the speed it was last asked for.
    private final PIDFCoefficients sentPidf = new PIDFCoefficients(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    private double targetVelocity = 0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        if (!enabled) {
            return;
        }
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The Control Hub runs the PIDF loop itself; we only give it the numbers.
        sendPidfIfChanged();

        // The right servo faces the other way, so it is reversed: both pull balls in together.
        leftServo.setPower(0);
        rightServo.setPower(0);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /*
     * Call once per loop. Powers are -1 to 1: + pulls balls in, - pushes them out.
     * The roller and the corner servos get separate powers, because the Auto only runs the roller.
     */
    public void run(double rollerPower, double servoPower) {
        if (!enabled) {
            return;
        }
        sendPidfIfChanged();
        targetVelocity = Range.clip(rollerPower, -1, 1) * MAX_TICKS_PER_SEC;
        intake.setVelocity(targetVelocity);
        double servo = Range.clip(servoPower, -1, 1);
        leftServo.setPower(servo);
        rightServo.setPower(servo);
    }

    // Sends PIDF to the motor when a number changed (in FTC Dashboard), not every loop.
    private void sendPidfIfChanged() {
        if (PIDF.p == sentPidf.p && PIDF.i == sentPidf.i && PIDF.d == sentPidf.d && PIDF.f == sentPidf.f) {
            return;
        }
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        sentPidf.p = PIDF.p;
        sentPidf.i = PIDF.i;
        sentPidf.d = PIDF.d;
        sentPidf.f = PIDF.f;
    }

    public void stop() {
        run(0, 0);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (enabled) {
            telemetry.addData("intake velocity", intake.getVelocity());
            telemetry.addData("intake target", targetVelocity);
        }
    }
}
