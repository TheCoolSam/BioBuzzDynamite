package org.firstinspires.ftc.teamcode.math;

/**
 * Finite-difference rate with an optional first-order low-pass.
 * The first sample never invents a rate. Nonfinite input or a non-positive
 * interval discards history. {@code tauSec = 0} is the raw difference.
 */
public final class FilteredDerivative {
    public final double tauSec;
    private double previousValue = Double.NaN;
    private double previousTime = Double.NaN;
    private double rate = Double.NaN;

    public FilteredDerivative(double tauSec) {
        if (!Double.isFinite(tauSec) || tauSec < 0) {
            throw new IllegalArgumentException("Non-negative filter time constant required");
        }
        this.tauSec = tauSec;
    }

    public void reset() {
        previousValue = previousTime = rate = Double.NaN;
    }

    public double update(double value, double timeSec) {
        if (!Double.isFinite(value) || !Double.isFinite(timeSec)) {
            reset();
            return Double.NaN;
        }
        if (!Double.isFinite(previousValue) || !Double.isFinite(previousTime)) {
            previousValue = value;
            previousTime = timeSec;
            rate = Double.NaN;
            return Double.NaN;
        }
        double dt = timeSec - previousTime;
        if (dt <= 0) {
            reset();
            return Double.NaN;
        }
        double raw = (value - previousValue) / dt;
        previousValue = value;
        previousTime = timeSec;
        if (tauSec == 0 || !Double.isFinite(rate)) {
            rate = raw;
            return rate;
        }
        double alpha = dt / (tauSec + dt);
        rate += alpha * (raw - rate);
        return rate;
    }

    public double rate() {
        return rate;
    }
}
