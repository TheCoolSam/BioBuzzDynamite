package org.firstinspires.ftc.teamcode.turret;

/** All wiring/calibration decisions live here. Defaults intentionally prohibit motion. */
public final class TurretHardwareConfig {
    public enum Backend { UNSELECTED, ANALOG_ABSOLUTE, PWM_ABSOLUTE }
    public static final Backend BACKEND = Backend.UNSELECTED;
    public static final String SERVO_NAME = "turretYaw";
    public static final boolean CR_MODE_AND_NEUTRAL_VERIFIED = false;
    // Set from loaded Axon CR firmware and measured pulse/stop behavior, never inferred.
    public static final AxonCrCalibration CR_CALIBRATION = null;
    // Measurement-only pulse window. NaN refuses CR_DEADBAND output; not a runtime calibration.
    public static final boolean CR_MEASURE_CLEARED = false;
    public static final double CR_MEASURE_START_US = Double.NaN;
    public static final double CR_MEASURE_MIN_US = Double.NaN;
    public static final double CR_MEASURE_MAX_US = Double.NaN;
    public static final double CR_MEASURE_FRAME_US = Double.NaN;
    public static final double CR_MEASURE_STEP_US = 5;
    public static final double CR_MEASURE_STATIONARY_RATE_RAD_PER_SEC = Math.toRadians(2);
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
    // Exclusive of analog rails. Near-0V zero is not a valid band by itself.
    public static final double ANALOG_VALID_MIN_VOLTS = Double.NaN;
    public static final double ANALOG_VALID_MAX_VOLTS = Double.NaN;
    public static final double ANALOG_STUCK_EPSILON_VOLTS = Double.NaN;
    public static final int ANALOG_STUCK_SAMPLES = 0;
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
    // Software direction/stall windows, not measured mechanism constants.
    public static final double DIRECTION_TRIP_MIN_POWER = 0.02;
    public static final double DIRECTION_OPPOSE_SEC = 0.15;
    public static final double DIRECTION_STALL_SEC = 0.35;
    public static final double DIRECTION_STALL_RATE_RAD_PER_SEC = Math.toRadians(3);

    private TurretHardwareConfig() { }

    /**
     * Sign-test magnitude from the measured CR deadband. Uncalibrated config
     * objects used by desktop fixtures keep {@link #SIGN_TEST_POWER}.
     */
    public static double effectiveSignTestPower() {
        if (CR_CALIBRATION == null) return SIGN_TEST_POWER;
        return CR_CALIBRATION.validatedSignTestPower(SIGN_TEST_POWER, BENCH_MAX_POWER);
    }

    public static String signTestRefusal() {
        if (CR_CALIBRATION == null || !CR_MODE_AND_NEUTRAL_VERIFIED) {
            return "CR MODE / NEUTRAL NOT VERIFIED";
        }
        if (!CR_CALIBRATION.deadbandMeasured()) return "CR DEADBAND NOT MEASURED";
        double power = CR_CALIBRATION.validatedSignTestPower(SIGN_TEST_POWER, BENCH_MAX_POWER);
        if (Double.isFinite(power)) return "";
        return "SIGN TEST POWER INSIDE CR DEADBAND / EXCEEDS BENCH CAP";
    }

    public static boolean crMeasureWindowConfigured() {
        return CR_MEASURE_CLEARED
                && Double.isFinite(CR_MEASURE_START_US)
                && Double.isFinite(CR_MEASURE_MIN_US)
                && Double.isFinite(CR_MEASURE_MAX_US)
                && Double.isFinite(CR_MEASURE_FRAME_US)
                && Double.isFinite(CR_MEASURE_STEP_US)
                && CR_MEASURE_STEP_US > 0
                && CR_MEASURE_MIN_US > 0
                && CR_MEASURE_MAX_US > CR_MEASURE_MIN_US
                && CR_MEASURE_START_US >= CR_MEASURE_MIN_US
                && CR_MEASURE_START_US <= CR_MEASURE_MAX_US
                && CR_MEASURE_FRAME_US > CR_MEASURE_MAX_US;
    }
}
