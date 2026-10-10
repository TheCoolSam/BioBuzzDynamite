package org.firstinspires.ftc.teamcode.turret;

/** All wiring/calibration decisions live here. Defaults intentionally prohibit motion. */
public final class TurretHardwareConfig {
    public enum Backend { UNSELECTED, ANALOG_ABSOLUTE, PWM_ABSOLUTE }
    public static final Backend BACKEND = Backend.UNSELECTED;
    public static final String MOTOR_NAME = "turretMotor";
    public static final String ENCODER_NAME = "turretEncoder";
    public static final int MOTOR_SIGN = 1; // SDK FORWARD, then multiply normalized power
    public static final int ENCODER_SIGN = 1;
    // Zero = signed raw angle at physical forward. NaN means NOT CALIBRATED.
    public static final double ENCODER_ZERO_OFFSET_RAD = Double.NaN;
    public static final boolean WIRING_AND_TRANSFER_CONFIRMED = false;
    public static final boolean OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED = false;
    // Must come from verified device transfer function, not AnalogInput.getMaxVoltage().
    public static final double ANALOG_ZERO_TURN_VOLTS = Double.NaN;
    public static final double ANALOG_FULL_TURN_VOLTS = Double.NaN;
    public static final boolean ENCODER_SIGN_VERIFIED = false;
    public static final boolean MOTOR_SIGN_VERIFIED = false;
    public static final boolean LIMITS_CONFIRMED = false;
    public static final boolean SMALL_STEPS_PASSED = false;
    public static final boolean BENCH_POWER_CONFIRMED = false;
    public static final boolean CLOSED_LOOP_ENABLED = false;
    public static final boolean COMPETITION_TUNING_CONFIRMED = false;
    // Bench-only placeholders requiring physical confirmation. No competition tuning.
    public static final double BENCH_TEST_MIN_RAD = Math.toRadians(-90);
    public static final double BENCH_TEST_MAX_RAD = Math.toRadians(90);
    public static final double BENCH_MAX_POWER = 0.15;
    public static final double SIGN_TEST_POWER = 0.03;
    public static final double BENCH_KP = 0.15; // power/rad, untuned test value
    public static final double BENCH_KD = 0;
    public static final double BENCH_KV = 0;
    // Plausibility ceiling only; not a characterized safe operating speed.
    public static final double SENSOR_MAX_RATE_RAD_PER_SEC = Math.toRadians(360);
    public static final double MIN_DT_SEC = 0.0001;
    public static final double MAX_DT_SEC = 0.2;
    private TurretHardwareConfig() { }
}
