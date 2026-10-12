package org.firstinspires.ftc.teamcode.turret;

/** Independent output gate, used by real I/O even when a caller bypasses the bench controller. */
public final class TurretOutputSafety {
    public final double minRad, maxRad, cap;
    public String reason = "NOT ARMED";
    public boolean positiveLimit, negativeLimit;
    private final boolean configured;
    private final double physicalMinRad, physicalMaxRad;
    private final double tripMinPower, opposeSec, stallSec, stallRate;
    private double tripTimer;
    private boolean directionFault;
    public TurretOutputSafety(double physicalMin, double physicalMax,
            double operatingMin, double operatingMax, double benchMin, double benchMax, double cap) {
        this(physicalMin, physicalMax, operatingMin, operatingMax, benchMin, benchMax, cap,
                TurretHardwareConfig.DIRECTION_TRIP_MIN_POWER,
                TurretHardwareConfig.DIRECTION_OPPOSE_SEC,
                TurretHardwareConfig.DIRECTION_STALL_SEC,
                TurretHardwareConfig.DIRECTION_STALL_RATE_RAD_PER_SEC);
    }
    public TurretOutputSafety(double physicalMin, double physicalMax,
            double operatingMin, double operatingMax, double benchMin, double benchMax, double cap,
            double tripMinPower, double opposeSec, double stallSec, double stallRate) {
        this.minRad = Math.max(physicalMin, Math.max(operatingMin, benchMin));
        this.maxRad = Math.min(physicalMax, Math.min(operatingMax, benchMax));
        this.cap = cap;
        physicalMinRad = physicalMin; physicalMaxRad = physicalMax;
        this.tripMinPower = tripMinPower;
        this.opposeSec = opposeSec;
        this.stallSec = stallSec;
        this.stallRate = stallRate;
        configured = Double.isFinite(physicalMin) && Double.isFinite(physicalMax)
                && Double.isFinite(operatingMin) && Double.isFinite(operatingMax)
                && Double.isFinite(benchMin) && Double.isFinite(benchMax)
                && physicalMin <= operatingMin && operatingMin <= benchMin
                && benchMin < benchMax && benchMax <= operatingMax && operatingMax <= physicalMax
                && Double.isFinite(cap) && cap > 0 && cap <= 1
                && Double.isFinite(tripMinPower) && tripMinPower > 0
                && Double.isFinite(opposeSec) && opposeSec > 0
                && Double.isFinite(stallSec) && stallSec > 0
                && Double.isFinite(stallRate) && stallRate >= 0;
    }
    public double apply(double demand, double angle, boolean sensorValid, double dt,
                        boolean armed, boolean deadman, boolean emergencyStop) {
        return apply(demand, angle, Double.NaN, sensorValid, dt, armed, deadman, emergencyStop);
    }
    public double apply(double demand, double angle, double velocity, boolean sensorValid, double dt,
                        boolean armed, boolean deadman, boolean emergencyStop) {
        positiveLimit = Double.isFinite(angle) && angle >= maxRad;
        negativeLimit = Double.isFinite(angle) && angle <= minRad;
        reason = "";
        if (emergencyStop) reason = "EMERGENCY STOP";
        else if (directionFault) reason = "WRONG DIRECTION / STALL TRIP";
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
        if (!reason.isEmpty()) {
            if (!armed || emergencyStop || !deadman) tripTimer = 0;
            return 0;
        }
        if (Math.abs(demand) > cap) reason = "BENCH POWER CAPPED";
        double output = Math.max(-cap, Math.min(cap, demand));
        if (updateDirectionTrip(output, velocity, dt)) {
            reason = "WRONG DIRECTION / STALL TRIP";
            return 0;
        }
        return output;
    }
    private boolean updateDirectionTrip(double output, double velocity, double dt) {
        if (!Double.isFinite(velocity) || !Double.isFinite(output) || !TurretEncoderSample.validDt(dt)
                || Math.abs(output) < tripMinPower) {
            tripTimer = 0;
            return directionFault;
        }
        boolean opposite = output * velocity < 0 && Math.abs(velocity) >= stallRate;
        boolean stall = Math.abs(velocity) < stallRate;
        if (!opposite && !stall) {
            tripTimer = 0;
            return directionFault;
        }
        tripTimer += dt;
        double window = opposite ? opposeSec : stallSec;
        if (tripTimer >= window) {
            directionFault = true;
            tripTimer = 0;
        }
        return directionFault;
    }
    public boolean directionFault() { return directionFault; }
    public void clearFaults() {
        directionFault = false;
        tripTimer = 0;
        if (reason.startsWith("WRONG DIRECTION")) reason = "NOT ARMED";
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
