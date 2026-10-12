package org.firstinspires.ftc.teamcode.turret;

/** Pure operator state machine. Direct position steps use untuned P only, D=V=0. */
public final class TurretBenchSession {
    public enum Mode { SENSOR_ONLY, CR_DEADBAND, SIGN_TEST, SMALL_STEPS, SWEEP }
    public Mode mode = Mode.SENSOR_ONLY;
    public boolean armed, emergencyStop, measureActive;
    public double targetRad, rawPower, errorRad = Double.NaN;
    public double measurePulseUs = Double.NaN;
    public double suggestedNeutralUs = Double.NaN;
    public double suggestedDeadbandLowUs = Double.NaN;
    public double suggestedDeadbandHighUs = Double.NaN;
    public String reason = "SENSOR ONLY";
    public String crPaste = "";
    private boolean previousMode, previousArm, previousReset;
    private boolean previousLeft, previousRight, previousUp, previousDown;
    public static final class Input {
        public boolean nextMode, arm, reset, stop, deadman, left, right, up, down, center;
    }
    public static final class Readiness {
        public boolean crMeasure, openLoop, closedLoop, sweep;
        public String missing = "HARDWARE CHECKLIST INCOMPLETE";
        public String crMissing = "CR MEASURE WINDOW UNCONFIGURED";
        public String closedMissing = "CLOSED LOOP DISABLED / MOTOR SIGN NOT VERIFIED";
        public String sweepMissing = "SMALL STEP TESTS NOT CONFIRMED";
        public static Readiness configured() {
            Readiness r = new Readiness();
            boolean sensing = TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED
                    && TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED
                    && TurretHardwareConfig.ENCODER_SIGN_VERIFIED;
            r.crMeasure = sensing && TurretHardwareConfig.crMeasureWindowConfigured();
            boolean signOk = TurretHardwareConfig.CR_MODE_AND_NEUTRAL_VERIFIED
                    && TurretHardwareConfig.CR_CALIBRATION != null
                    && Double.isFinite(TurretHardwareConfig.CR_CALIBRATION.validatedSignTestPower(
                            TurretHardwareConfig.SIGN_TEST_POWER, TurretHardwareConfig.BENCH_MAX_POWER));
            r.openLoop = sensing
                    && TurretHardwareConfig.LIMITS_CONFIRMED
                    && TurretHardwareConfig.BENCH_POWER_CONFIRMED
                    && signOk;
            r.closedLoop = r.openLoop && TurretHardwareConfig.MOTOR_SIGN_VERIFIED
                    && TurretHardwareConfig.CLOSED_LOOP_ENABLED;
            r.sweep = r.closedLoop && TurretHardwareConfig.SMALL_STEPS_PASSED;
            if (!TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED) r.missing = "WIRING / TRANSFER NOT CONFIRMED";
            else if (!TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED) r.missing = "OUTPUT-SIDE 1:1 NOT CONFIRMED";
            else if (!TurretHardwareConfig.ENCODER_SIGN_VERIFIED) r.missing = "ENCODER SIGN NOT VERIFIED";
            else if (!TurretHardwareConfig.LIMITS_CONFIRMED) r.missing = "MEASURED LIMITS NOT CONFIRMED";
            else if (!TurretHardwareConfig.BENCH_POWER_CONFIRMED) r.missing = "BENCH POWER NOT CONFIRMED";
            else if (!TurretHardwareConfig.CR_MODE_AND_NEUTRAL_VERIFIED
                    || TurretHardwareConfig.CR_CALIBRATION == null) {
                r.missing = "CR MODE / NEUTRAL NOT VERIFIED";
            } else {
                String sign = TurretHardwareConfig.signTestRefusal();
                r.missing = sign.isEmpty() ? "HARDWARE CHECKLIST INCOMPLETE" : sign;
            }
            if (!TurretHardwareConfig.CR_MEASURE_CLEARED) r.crMissing = "CR MEASURE NOT CLEARED SAFE";
            else if (!TurretHardwareConfig.crMeasureWindowConfigured()) r.crMissing = "CR MEASURE START/WINDOW UNCONFIGURED";
            else if (!TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED
                    || !TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED
                    || !TurretHardwareConfig.ENCODER_SIGN_VERIFIED) {
                r.crMissing = r.missing;
            } else r.crMissing = "";
            return r;
        }
    }
    public double update(Input in, Readiness ready, TurretEncoderSample sample, double dt) {
        boolean neutral = !in.deadman && !in.left && !in.right && !in.up && !in.down;
        boolean modeEdge = in.nextMode && !previousMode;
        boolean armEdge = in.arm && !previousArm;
        boolean resetEdge = in.reset && !previousReset;
        boolean leftEdge = in.left && !previousLeft, rightEdge = in.right && !previousRight;
        boolean upEdge = in.up && !previousUp, downEdge = in.down && !previousDown;
        previousMode = in.nextMode; previousArm = in.arm; previousReset = in.reset;
        previousLeft = in.left; previousRight = in.right; previousUp = in.up; previousDown = in.down;
        rawPower = 0;
        measureActive = false;
        if (in.stop) { emergencyStop = true; armed = false; mode = Mode.SENSOR_ONLY; }
        if (resetEdge && neutral && !in.stop) {
            emergencyStop = false; armed = false; mode = Mode.SENSOR_ONLY;
        }
        if (modeEdge && neutral && !emergencyStop && !resetEdge) {
            mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            armed = false; targetRad = 0;
            if (mode == Mode.CR_DEADBAND) {
                measurePulseUs = TurretHardwareConfig.CR_MEASURE_START_US;
                suggestedNeutralUs = suggestedDeadbandLowUs = suggestedDeadbandHighUs = Double.NaN;
            }
        }
        boolean permitted = mode == Mode.CR_DEADBAND ? ready.crMeasure
                : mode == Mode.SIGN_TEST ? ready.openLoop
                : mode == Mode.SMALL_STEPS ? ready.closedLoop : mode == Mode.SWEEP && ready.sweep;
        if (!sample.valid || !TurretEncoderSample.validDt(dt) || !permitted || emergencyStop) armed = false;
        // A new edge with deadman released is mandatory after every fault/mode change.
        if (armEdge && !modeEdge && !resetEdge && neutral && permitted && sample.valid && TurretEncoderSample.validDt(dt)
                && !emergencyStop) armed = true;
        if (mode == Mode.SMALL_STEPS || mode == Mode.SWEEP) {
            double step = Math.toRadians(10);
            if (leftEdge) targetRad -= step;
            if (rightEdge) targetRad += step;
            if (upEdge) targetRad = mode == Mode.SMALL_STEPS ? Math.toRadians(20)
                    : TurretHardwareConfig.BENCH_TEST_MAX_RAD;
            if (downEdge) targetRad = mode == Mode.SMALL_STEPS ? Math.toRadians(-20)
                    : TurretHardwareConfig.BENCH_TEST_MIN_RAD;
            if (in.center) targetRad = 0;
            double low = Math.max(TurretHardwareConfig.BENCH_TEST_MIN_RAD,
                    mode == Mode.SMALL_STEPS ? Math.toRadians(-20) : Double.NEGATIVE_INFINITY);
            double high = Math.min(TurretHardwareConfig.BENCH_TEST_MAX_RAD,
                    mode == Mode.SMALL_STEPS ? Math.toRadians(20) : Double.POSITIVE_INFINITY);
            targetRad = Math.max(low, Math.min(high, targetRad));
        }
        if (mode == Mode.CR_DEADBAND) {
            if (!Double.isFinite(measurePulseUs)) measurePulseUs = TurretHardwareConfig.CR_MEASURE_START_US;
            if (leftEdge && Double.isFinite(measurePulseUs)) {
                measurePulseUs -= TurretHardwareConfig.CR_MEASURE_STEP_US;
            }
            if (rightEdge && Double.isFinite(measurePulseUs)) {
                measurePulseUs += TurretHardwareConfig.CR_MEASURE_STEP_US;
            }
            if (Double.isFinite(TurretHardwareConfig.CR_MEASURE_MIN_US)
                    && Double.isFinite(TurretHardwareConfig.CR_MEASURE_MAX_US)
                    && Double.isFinite(measurePulseUs)) {
                measurePulseUs = Math.max(TurretHardwareConfig.CR_MEASURE_MIN_US,
                        Math.min(TurretHardwareConfig.CR_MEASURE_MAX_US, measurePulseUs));
            }
            observeDeadband(sample);
        }
        errorRad = sample.valid ? targetRad - sample.angleRad : Double.NaN;
        if (emergencyStop) reason = "EMERGENCY STOP";
        else if (mode == Mode.SENSOR_ONLY) reason = "SENSOR ONLY";
        else if (!sample.valid) reason = sample.fault;
        else if (!TurretEncoderSample.validDt(dt)) reason = "LOOP DT INVALID";
        else if (!permitted) reason = mode == Mode.CR_DEADBAND ? ready.crMissing
                : !ready.openLoop ? ready.missing
                : !ready.closedLoop ? ready.closedMissing : ready.sweepMissing;
        else if (!armed) reason = "NOT ARMED: RELEASE RB / DPAD, PRESS A";
        else if (!in.deadman) reason = "DEADMAN RELEASED";
        else {
            reason = "";
            if (mode == Mode.CR_DEADBAND) {
                measureActive = TurretHardwareConfig.crMeasureWindowConfigured()
                        && Double.isFinite(measurePulseUs);
                if (!measureActive) reason = "CR MEASURE WINDOW UNCONFIGURED";
            } else if (mode == Mode.SIGN_TEST) {
                double signPower = TurretHardwareConfig.effectiveSignTestPower();
                if (TurretHardwareConfig.CR_CALIBRATION != null && !Double.isFinite(signPower)) {
                    reason = TurretHardwareConfig.signTestRefusal();
                } else {
                    if (!Double.isFinite(signPower)) signPower = TurretHardwareConfig.SIGN_TEST_POWER;
                    rawPower = in.left == in.right ? 0 : (in.right ? 1 : -1) * signPower;
                }
            } else rawPower = TurretHardwareConfig.BENCH_KP * errorRad;
        }
        return rawPower;
    }
    private void observeDeadband(TurretEncoderSample sample) {
        crPaste = formatCrPaste();
        if (!sample.valid || !Double.isFinite(measurePulseUs) || !Double.isFinite(sample.velocityRadPerSec)) return;
        double stationary = TurretHardwareConfig.CR_MEASURE_STATIONARY_RATE_RAD_PER_SEC;
        if (Math.abs(sample.velocityRadPerSec) < stationary) {
            suggestedNeutralUs = measurePulseUs;
        } else if (sample.velocityRadPerSec > 0) {
            suggestedDeadbandHighUs = measurePulseUs;
        } else {
            suggestedDeadbandLowUs = measurePulseUs;
        }
        crPaste = formatCrPaste();
    }
    public String formatCrPaste() {
        return "new AxonCrCalibration("
                + fmt(TurretHardwareConfig.CR_MEASURE_MIN_US) + ", "
                + fmt(suggestedDeadbandLowUs) + ", "
                + fmt(suggestedNeutralUs) + ", "
                + fmt(suggestedDeadbandHighUs) + ", "
                + fmt(TurretHardwareConfig.CR_MEASURE_MAX_US) + ", "
                + fmt(TurretHardwareConfig.CR_MEASURE_FRAME_US) + ", "
                + TurretHardwareConfig.MOTOR_SIGN
                + "); // paste after verifying rate vs pulse; MOTOR_SIGN unverified until SIGN_TEST";
    }
    private static String fmt(double value) {
        return Double.isFinite(value) ? String.format(java.util.Locale.US, "%.1f", value) : "Double.NaN";
    }
}
