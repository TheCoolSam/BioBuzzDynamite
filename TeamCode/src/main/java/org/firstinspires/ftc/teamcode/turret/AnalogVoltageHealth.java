package org.firstinspires.ftc.teamcode.turret;

/**
 * Disconnect and plausibility gate for analog volts. Rails and an unconfigured
 * valid band never become a legal transfer. Stuck/zero-noise is opt-in because
 * a stationary sensor can be quiet; leave epsilon NaN until a hand-turn or
 * unplug test justifies it.
 */
public final class AnalogVoltageHealth {
    public final double validMinVolts, validMaxVolts, stuckEpsilonVolts;
    public final int stuckSamples;
    private double lastRaw = Double.NaN;
    private int frozenCount;

    public AnalogVoltageHealth(double validMinVolts, double validMaxVolts,
            double stuckEpsilonVolts, int stuckSamples) {
        this.validMinVolts = validMinVolts;
        this.validMaxVolts = validMaxVolts;
        this.stuckEpsilonVolts = stuckEpsilonVolts;
        this.stuckSamples = stuckSamples;
    }

    public static AnalogVoltageHealth unconfigured() {
        return new AnalogVoltageHealth(Double.NaN, Double.NaN, Double.NaN, 0);
    }

    /** Valid band is the calibrated transfer interval; rails are still rejected. */
    public static AnalogVoltageHealth railsAndRange(double minVolts, double maxVolts) {
        return new AnalogVoltageHealth(minVolts, maxVolts, Double.NaN, 0);
    }

    public static AnalogVoltageHealth fromTurretConfig() {
        return new AnalogVoltageHealth(
                TurretHardwareConfig.ANALOG_VALID_MIN_VOLTS,
                TurretHardwareConfig.ANALOG_VALID_MAX_VOLTS,
                TurretHardwareConfig.ANALOG_STUCK_EPSILON_VOLTS,
                TurretHardwareConfig.ANALOG_STUCK_SAMPLES);
    }

    public void reset() {
        lastRaw = Double.NaN;
        frozenCount = 0;
    }

    /** Empty string when the voltage may be converted. Otherwise a fault. */
    public String check(double rawVolts, double adcMaxVolts) {
        if (!Double.isFinite(rawVolts) || !Double.isFinite(adcMaxVolts) || adcMaxVolts <= 0) {
            reset();
            return "ANALOG RANGE / TRANSFER INVALID";
        }
        if (rawVolts <= 0 || rawVolts >= adcMaxVolts) {
            reset();
            return "ANALOG DISCONNECT / RAIL";
        }
        if (!Double.isFinite(validMinVolts) || !Double.isFinite(validMaxVolts)
                || validMinVolts < 0 || validMaxVolts <= validMinVolts || validMaxVolts >= adcMaxVolts) {
            lastRaw = rawVolts;
            frozenCount = 0;
            return "ANALOG VALID BAND UNCONFIGURED";
        }
        if (rawVolts < validMinVolts || rawVolts > validMaxVolts) {
            lastRaw = rawVolts;
            frozenCount = 0;
            return "ANALOG OUTSIDE VALID BAND";
        }
        if (Double.isFinite(stuckEpsilonVolts) && stuckEpsilonVolts >= 0 && stuckSamples > 0) {
            if (Double.isFinite(lastRaw) && Math.abs(rawVolts - lastRaw) <= stuckEpsilonVolts) {
                frozenCount++;
                lastRaw = rawVolts;
                if (frozenCount >= stuckSamples) return "ANALOG STUCK / ZERO NOISE";
            } else {
                frozenCount = 1;
                lastRaw = rawVolts;
            }
        } else {
            lastRaw = rawVolts;
            frozenCount = 0;
        }
        return "";
    }
}
