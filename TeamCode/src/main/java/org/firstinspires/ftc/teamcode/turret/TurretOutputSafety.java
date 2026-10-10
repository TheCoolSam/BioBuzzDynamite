package org.firstinspires.ftc.teamcode.turret;

/** Independent output gate, used by real I/O even when a caller bypasses the bench controller. */
public final class TurretOutputSafety {
    public final double minRad, maxRad, cap;
    public String reason = "NOT ARMED";
    public boolean positiveLimit, negativeLimit;
    private final boolean configured;
    private final double physicalMinRad, physicalMaxRad;
    public TurretOutputSafety(double physicalMin, double physicalMax,
            double operatingMin, double operatingMax, double benchMin, double benchMax, double cap) {
        this.minRad = Math.max(physicalMin, Math.max(operatingMin, benchMin));
        this.maxRad = Math.min(physicalMax, Math.min(operatingMax, benchMax));
        this.cap = cap;
        physicalMinRad = physicalMin; physicalMaxRad = physicalMax;
        configured = Double.isFinite(physicalMin) && Double.isFinite(physicalMax)
                && Double.isFinite(operatingMin) && Double.isFinite(operatingMax)
                && Double.isFinite(benchMin) && Double.isFinite(benchMax)
                && physicalMin <= operatingMin && operatingMin <= benchMin
                && benchMin < benchMax && benchMax <= operatingMax && operatingMax <= physicalMax
                && Double.isFinite(cap) && cap > 0 && cap <= 1;
    }
    public double apply(double demand, double angle, boolean sensorValid, double dt,
                        boolean armed, boolean deadman, boolean emergencyStop) {
        positiveLimit = Double.isFinite(angle) && angle >= maxRad;
        negativeLimit = Double.isFinite(angle) && angle <= minRad;
        reason = "";
        if (emergencyStop) reason = "EMERGENCY STOP";
        else if (!configured) reason = "LIMITS / CAP INVALID";
        else if (!sensorValid || !Double.isFinite(angle)) reason = "ENCODER INVALID";
        // A measured angle outside confirmed mechanical travel is an impossible reading,
        // not permission to choose a recovery direction across the ±pi discontinuity.
        else if (angle < physicalMinRad || angle > physicalMaxRad) reason = "ANGLE OUTSIDE PHYSICAL RANGE";
        else if (!TurretEncoderSample.validDt(dt)) reason = "LOOP DT INVALID";
        else if (!armed) reason = "NOT ARMED";
        else if (!deadman) reason = "DEADMAN RELEASED";
        else if (!Double.isFinite(demand)) reason = "COMMAND INVALID";
        else if (positiveLimit && demand > 0) reason = "POSITIVE BENCH / OPERATING LIMIT";
        else if (negativeLimit && demand < 0) reason = "NEGATIVE BENCH / OPERATING LIMIT";
        if (!reason.isEmpty()) return 0;
        if (Math.abs(demand) > cap) reason = "BENCH POWER CAPPED";
        return Math.max(-cap, Math.min(cap, demand));
    }
    public static TurretOutputSafety benchDefaults() {
        return new TurretOutputSafety(TurretConstants.PHYSICAL_MIN_RAD, TurretConstants.PHYSICAL_MAX_RAD,
                TurretConstants.OPERATING_MIN_RAD, TurretConstants.OPERATING_MAX_RAD,
                TurretHardwareConfig.BENCH_TEST_MIN_RAD, TurretHardwareConfig.BENCH_TEST_MAX_RAD,
                TurretHardwareConfig.BENCH_MAX_POWER);
    }
    public double physicalMin() { return physicalMinRad; }
    public double physicalMax() { return physicalMaxRad; }
}
