package org.firstinspires.ftc.teamcode.turret;

/** Pure operator state machine. Direct position steps use untuned P only, D=V=0. */
public final class TurretBenchSession {
    public enum Mode { SENSOR_ONLY, SIGN_TEST, SMALL_STEPS, SWEEP }
    public Mode mode = Mode.SENSOR_ONLY;
    public boolean armed, emergencyStop;
    public double targetRad, rawPower, errorRad = Double.NaN;
    public String reason = "SENSOR ONLY";
    private boolean previousMode, previousArm, previousReset;
    private boolean previousLeft, previousRight, previousUp, previousDown;
    public static final class Input {
        public boolean nextMode, arm, reset, stop, deadman, left, right, up, down, center;
    }
    public static final class Readiness {
        public boolean openLoop, closedLoop, sweep;
        public String missing = "HARDWARE CHECKLIST INCOMPLETE";
        public String closedMissing = "CLOSED LOOP DISABLED / MOTOR SIGN NOT VERIFIED";
        public String sweepMissing = "SMALL STEP TESTS NOT CONFIRMED";
        public static Readiness configured() {
            Readiness r = new Readiness();
            r.openLoop = TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED
                    && TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED
                    && TurretHardwareConfig.ENCODER_SIGN_VERIFIED && TurretHardwareConfig.LIMITS_CONFIRMED
                    && TurretHardwareConfig.BENCH_POWER_CONFIRMED;
            r.closedLoop = r.openLoop && TurretHardwareConfig.MOTOR_SIGN_VERIFIED
                    && TurretHardwareConfig.CLOSED_LOOP_ENABLED;
            r.sweep = r.closedLoop && TurretHardwareConfig.SMALL_STEPS_PASSED;
            if (!TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED) r.missing = "WIRING / TRANSFER NOT CONFIRMED";
            else if (!TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED) r.missing = "OUTPUT-SIDE 1:1 NOT CONFIRMED";
            else if (!TurretHardwareConfig.ENCODER_SIGN_VERIFIED) r.missing = "ENCODER SIGN NOT VERIFIED";
            else if (!TurretHardwareConfig.LIMITS_CONFIRMED) r.missing = "MEASURED LIMITS NOT CONFIRMED";
            else if (!TurretHardwareConfig.BENCH_POWER_CONFIRMED) r.missing = "BENCH POWER NOT CONFIRMED";
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
        if (in.stop) { emergencyStop = true; armed = false; mode = Mode.SENSOR_ONLY; }
        if (resetEdge && neutral && !in.stop) {
            emergencyStop = false; armed = false; mode = Mode.SENSOR_ONLY;
        }
        if (modeEdge && neutral && !emergencyStop && !resetEdge) {
            mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            armed = false; targetRad = 0;
        }
        boolean permitted = mode == Mode.SIGN_TEST ? ready.openLoop
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
        errorRad = sample.valid ? targetRad - sample.angleRad : Double.NaN;
        if (emergencyStop) reason = "EMERGENCY STOP";
        else if (mode == Mode.SENSOR_ONLY) reason = "SENSOR ONLY";
        else if (!sample.valid) reason = sample.fault;
        else if (!TurretEncoderSample.validDt(dt)) reason = "LOOP DT INVALID";
        else if (!permitted) reason = !ready.openLoop ? ready.missing
                : !ready.closedLoop ? ready.closedMissing : ready.sweepMissing;
        else if (!armed) reason = "NOT ARMED: RELEASE RB / DPAD, PRESS A";
        else if (!in.deadman) reason = "DEADMAN RELEASED";
        else {
            reason = "";
            if (mode == Mode.SIGN_TEST) rawPower = in.left == in.right ? 0
                    : (in.right ? 1 : -1) * TurretHardwareConfig.SIGN_TEST_POWER;
            else rawPower = TurretHardwareConfig.BENCH_KP * errorRad;
        }
        return rawPower;
    }
}
