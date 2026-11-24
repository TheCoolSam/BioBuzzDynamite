package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * The two flywheel motors driven as one: every mode, PIDF, power and velocity command goes to both.
 * Speed is read from the first motor only.
 *
 * <p>The second motor is optional. If it is missing from the robot configuration, the first motor
 * runs alone and {@link #status()} says so.
 *
 * <p>Safety: in velocity mode each motor closes its own loop on its own encoder. If the second
 * motor's encoder reads near zero while the first is clearly spinning (encoder cable unplugged),
 * the second motor would run flat out, so it is switched off for the rest of the OpMode.
 */
public final class FlywheelPair {
    /** Robot-configuration name of the second flywheel motor. */
    public static final String SECOND_NAME = "flywheel2";
    /**
     * True when the second motor must spin the opposite way to the first to drive the flywheel
     * forward (motors facing each other). In the CAD both motors sit on the same side, face the
     * same way and drive the same 3-pulley GT2 belt, so they spin the same direction (false).
     * If the two motors fight (loud, hot, slow), flip this.
     */
    public static boolean SECOND_OPPOSITE = false;

    /**
     * Velocity control runs here in software instead of in the hub's PIDF. The hub loop has no battery
     * compensation, so as the battery sags and the motor warms the flywheel slowly loses speed.
     * The gains keep the hub's units, so stored and dashboard P/I/D/F numbers still mean the same:
     * power = (F * target + P * error + I * integral(error) + D * d(error)/dt) / 32767, with F, P and D
     * scaled by 12 V / battery volts. Set false to go back to the hub's own velocity PIDF.
     */
    public static boolean SOFTWARE_VELOCITY = true;
    /** Most power the integral term may add or remove. */
    public static double I_LIMIT = 0.15;
    /** The integral only builds when the error is smaller than this fraction of the target (no spin-up wind-up). */
    public static double I_ZONE = 0.15;

    private static final double MISMATCH_SEC = 0.5;
    private static final double HUB_FULL_SCALE = 32767.0;
    private static final double NOMINAL_VOLTS = 12.0;

    private final DcMotorEx first;
    private final DcMotorEx second;
    private final String secondName;
    private final List<VoltageSensor> batteries;
    private boolean secondDisabled;
    private boolean velocityMode;
    private double velocityTarget;
    private long mismatchSinceNanos = -1;
    private String status;

    private PIDFCoefficients gains = new PIDFCoefficients(15.0, 15.0, 0.0, 12.5);
    private double integral;
    private double lastError = Double.NaN;
    private long lastUpdateNanos = -1;
    private double lastPower;
    private double volts = NOMINAL_VOLTS;
    private long lastVoltsNanos = -1;

    private FlywheelPair(DcMotorEx first, DcMotorEx second, String secondName, List<VoltageSensor> batteries) {
        this.first = first;
        this.second = second;
        this.secondName = secondName;
        this.batteries = batteries;
        if (second != null) {
            status = "two motors";
        } else if (secondName == null || secondName.trim().isEmpty()) {
            status = "single motor";
        } else {
            status = "single motor (no \"" + secondName + "\" configured)";
        }
    }

    /** Maps both motors; the first is required, the second is used when it exists. */
    public static FlywheelPair map(HardwareMap map, String firstName, String secondName) {
        DcMotorEx a = map.get(DcMotorEx.class, firstName);
        DcMotorEx b = null;
        if (secondName != null && !secondName.trim().isEmpty()) {
            try {
                b = map.get(DcMotorEx.class, secondName);
            } catch (RuntimeException missing) {
                b = null;
            }
        }
        List<VoltageSensor> batteries = new ArrayList<VoltageSensor>();
        for (VoltageSensor v : map.voltageSensor) batteries.add(v);
        return new FlywheelPair(a, b, secondName, batteries);
    }

    public DcMotorEx first() { return first; }
    public boolean hasSecond() { return second != null && !secondDisabled; }
    public String status() { return status; }

    /** Both motors, for code that stops every motor in a list. */
    public List<DcMotor> motors() {
        List<DcMotor> list = new ArrayList<DcMotor>();
        list.add(first);
        if (second != null) list.add(second);
        return list;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        first.setDirection(direction);
        if (second != null) second.setDirection(SECOND_OPPOSITE ? opposite(direction) : direction);
    }

    public void setMode(DcMotor.RunMode mode) {
        velocityMode = mode == DcMotor.RunMode.RUN_USING_ENCODER;
        // Software velocity drives raw power; the encoder is still read for speed.
        DcMotor.RunMode hubMode = velocityMode && SOFTWARE_VELOCITY ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode;
        first.setMode(hubMode);
        if (second != null) second.setMode(hubMode);
        mismatchSinceNanos = -1;
        resetLoop();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        first.setZeroPowerBehavior(behavior);
        if (second != null) second.setZeroPowerBehavior(behavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidf) {
        gains = new PIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        if (SOFTWARE_VELOCITY && mode == DcMotor.RunMode.RUN_USING_ENCODER) return;
        first.setPIDFCoefficients(mode, pidf);
        if (second != null) second.setPIDFCoefficients(mode, pidf);
    }

    public void setPower(double power) {
        velocityTarget = 0;
        resetLoop();
        lastPower = power;
        first.setPower(power);
        if (second != null) second.setPower(secondDisabled ? 0 : power);
    }

    /**
     * Ticks per second, applied to both motors. In software mode this must be called every loop:
     * each call is one step of the speed controller.
     */
    public void setVelocity(double ticksPerSecond) {
        velocityTarget = ticksPerSecond;
        if (SOFTWARE_VELOCITY) {
            double power = step(ticksPerSecond);
            first.setPower(power);
            if (second != null) second.setPower(secondDisabled ? 0 : power);
            if (second != null && !secondDisabled) checkSecondEncoder();
            return;
        }
        first.setVelocity(ticksPerSecond);
        if (second == null) return;
        if (secondDisabled) { second.setPower(0); return; }
        second.setVelocity(ticksPerSecond);
        checkSecondEncoder();
    }

    /** One controller step: feed-forward + P + bounded I (+ D), all in hub units, battery compensated. */
    private double step(double target) {
        long now = System.nanoTime();
        if (!(target > 0) || !Double.isFinite(target)) { resetLoop(); lastPower = 0; return 0; }
        double dt = lastUpdateNanos < 0 ? 0 : Math.min(0.1, (now - lastUpdateNanos) * 1e-9);
        lastUpdateNanos = now;
        double speed = Math.abs(first.getVelocity());
        double error = target - speed;
        double scale = NOMINAL_VOLTS / batteryVolts(now);
        if (dt > 0 && Math.abs(error) < I_ZONE * target) {
            integral += gains.i * error * dt / HUB_FULL_SCALE;
            integral = Math.max(-I_LIMIT, Math.min(I_LIMIT, integral));
        }
        double derivative = 0;
        if (dt > 0 && Double.isFinite(lastError)) derivative = (error - lastError) / dt;
        lastError = error;
        double power = (gains.f * target + gains.p * error + gains.d * derivative) * scale / HUB_FULL_SCALE + integral;
        // Never brake the flywheel with reverse power: let it coast down to a lower target.
        lastPower = Math.max(0.0, Math.min(1.0, power));
        return lastPower;
    }

    private double batteryVolts(long now) {
        if (lastVoltsNanos >= 0 && now - lastVoltsNanos < 100_000_000L) return volts;
        lastVoltsNanos = now;
        double best = 0;
        for (VoltageSensor v : batteries) {
            try {
                double reading = v.getVoltage();
                if (reading > best) best = reading;
            } catch (RuntimeException ignored) { }
        }
        // Ignore glitches; filter so one bad read cannot jump the output.
        if (best > 7.0 && best < 15.5) volts += 0.3 * (best - volts);
        return volts;
    }

    private void resetLoop() {
        integral = 0;
        lastError = Double.NaN;
        lastUpdateNanos = -1;
    }

    /** Power the flywheel is being driven at (software loop output, or what the hub reports). */
    public double getPower() { return SOFTWARE_VELOCITY && velocityMode ? lastPower : first.getPower(); }

    /** Filtered battery voltage the speed loop is compensating for. */
    public double getBatteryVolts() { return batteryVolts(System.nanoTime()); }

    /** Integral term's current contribution, as power. */
    public double getIntegralPower() { return integral; }

    /** Speed of the flywheel in ticks per second, from the first motor's encoder. */
    public double getVelocity() { return first.getVelocity(); }

    /** Total current of both motors, in amps. */
    public double getCurrentAmps() {
        double amps = first.getCurrent(CurrentUnit.AMPS);
        if (second != null) amps += second.getCurrent(CurrentUnit.AMPS);
        return amps;
    }

    public boolean isOverCurrent() {
        return first.isOverCurrent() || (second != null && second.isOverCurrent());
    }

    private void checkSecondEncoder() {
        if (!velocityMode || velocityTarget <= 0) { mismatchSinceNanos = -1; return; }
        double a = Math.abs(first.getVelocity());
        double b = Math.abs(second.getVelocity());
        boolean suspicious = a > 0.5 * velocityTarget && b < 0.25 * a;
        long now = System.nanoTime();
        if (!suspicious) { mismatchSinceNanos = -1; return; }
        if (mismatchSinceNanos < 0) { mismatchSinceNanos = now; return; }
        if ((now - mismatchSinceNanos) * 1e-9 >= MISMATCH_SEC) {
            secondDisabled = true;
            second.setPower(0);
            status = "SECOND MOTOR OFF: \"" + secondName + "\" encoder reads ~0 while the first motor spins "
                    + "(check its encoder cable, then restart the OpMode)";
        }
    }

    private static DcMotorSimple.Direction opposite(DcMotorSimple.Direction d) {
        return d == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
