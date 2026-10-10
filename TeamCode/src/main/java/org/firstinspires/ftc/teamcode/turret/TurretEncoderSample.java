package org.firstinspires.ftc.teamcode.turret;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** Pure conversion and wrap-safe differentiation. Invalid positions remain NaN. */
public final class TurretEncoderSample {
    public double raw = Double.NaN, unoffsetRad = Double.NaN;
    public double angleRad = Double.NaN, velocityRadPerSec = Double.NaN;
    public String units = "unavailable", fault = "ABSOLUTE ACQUISITION UNSELECTED";
    public boolean valid;
    private double previousAngle = Double.NaN;

    public void update(AbsoluteTurretSensor.Reading reading, double dt, int sign,
                       double zeroOffset, boolean outputSideConfirmed, double maxRate) {
        valid = false;
        raw = reading.raw; units = reading.units;
        angleRad = velocityRadPerSec = unoffsetRad = Double.NaN;
        fault = reading.fault;
        if (!fault.isEmpty() || !Double.isFinite(reading.turns)
                || reading.turns < 0 || reading.turns >= 1) {
            fail(fault.isEmpty() ? "ABSOLUTE ANGLE INVALID" : fault); return;
        }
        if (sign != 1 && sign != -1) { fail("ENCODER SIGN INVALID"); return; }
        unoffsetRad = sign * reading.turns * AngleUtil.TAU;
        if (!outputSideConfirmed) { fail("OUTPUT-SIDE 1:1 SENSING NOT CONFIRMED"); return; }
        if (!Double.isFinite(zeroOffset)) { fail("NOT CALIBRATED"); return; }
        if (!validDt(dt)) { fail("LOOP DT INVALID"); return; }
        angleRad = AngleUtil.wrapRadians(unoffsetRad - zeroOffset);
        // First sample supplies angle immediately but no invented velocity; establish baseline.
        if (!Double.isFinite(previousAngle)) {
            previousAngle = angleRad;
            fault = "VELOCITY BASELINE REQUIRED"; return;
        }
        velocityRadPerSec = AngleUtil.wrapRadians(angleRad - previousAngle) / dt;
        previousAngle = angleRad;
        if (!Double.isFinite(maxRate) || maxRate <= 0
                || !Double.isFinite(velocityRadPerSec) || Math.abs(velocityRadPerSec) > maxRate) {
            fail("ENCODER RATE IMPLAUSIBLE"); return;
        }
        valid = true; fault = "";
    }
    public void fail(String reason) {
        valid = false; fault = reason;
        angleRad = velocityRadPerSec = previousAngle = Double.NaN;
    }
    public static boolean validDt(double dt) {
        return Double.isFinite(dt) && dt >= TurretHardwareConfig.MIN_DT_SEC
                && dt <= TurretHardwareConfig.MAX_DT_SEC;
    }
}
