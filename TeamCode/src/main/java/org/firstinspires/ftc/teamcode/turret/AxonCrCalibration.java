package org.firstinspires.ftc.teamcode.turret;

/** Measured SDK power-to-pulse calibration. Zero demand always writes the verified neutral. */
public final class AxonCrCalibration {
    public final double lowUs, deadbandLowUs, neutralUs, deadbandHighUs, highUs, frameUs;
    public final int sign;

    public AxonCrCalibration(double lowUs, double neutralUs, double highUs, double frameUs, int sign) {
        this(lowUs, Double.NaN, neutralUs, Double.NaN, highUs, frameUs, sign);
    }

    public AxonCrCalibration(double lowUs, double deadbandLowUs, double neutralUs, double deadbandHighUs,
            double highUs, double frameUs, int sign) {
        if (!Double.isFinite(lowUs) || !Double.isFinite(neutralUs) || !Double.isFinite(highUs)
                || !Double.isFinite(frameUs) || lowUs <= 0 || neutralUs <= lowUs || highUs <= neutralUs
                || frameUs <= highUs || (sign != 1 && sign != -1)) {
            throw new IllegalArgumentException("Measured CR endpoints, neutral, frame and sign required");
        }
        boolean deadband = Double.isFinite(deadbandLowUs) || Double.isFinite(deadbandHighUs);
        if (deadband && (!Double.isFinite(deadbandLowUs) || !Double.isFinite(deadbandHighUs)
                || deadbandLowUs < lowUs || deadbandHighUs > highUs
                || deadbandLowUs > neutralUs || deadbandHighUs < neutralUs)) {
            throw new IllegalArgumentException("Measured CR deadband must sit around neutral inside endpoints");
        }
        this.lowUs = lowUs;
        this.deadbandLowUs = deadband ? deadbandLowUs : Double.NaN;
        this.neutralUs = neutralUs;
        this.deadbandHighUs = deadband ? deadbandHighUs : Double.NaN;
        this.highUs = highUs;
        this.frameUs = frameUs;
        this.sign = sign;
    }

    public boolean deadbandMeasured() {
        return Double.isFinite(deadbandLowUs) && Double.isFinite(deadbandHighUs);
    }

    public double pulseUs(double demand) {
        if (!Double.isFinite(demand) || Math.abs(demand) > 1) {
            throw new IllegalArgumentException("Bounded CR demand required");
        }
        double signed = demand * sign;
        return neutralUs + signed * (signed >= 0 ? highUs - neutralUs : neutralUs - lowUs);
    }

    public double sdkPower(double demand) {
        double pulse = pulseUs(demand);
        return 2 * (pulse - lowUs) / (highUs - lowUs) - 1;
    }

    public double demandForPulse(double pulseUs) {
        if (!Double.isFinite(pulseUs) || pulseUs < lowUs || pulseUs > highUs) return Double.NaN;
        double signed = pulseUs >= neutralUs
                ? (pulseUs - neutralUs) / (highUs - neutralUs)
                : (pulseUs - neutralUs) / (neutralUs - lowUs);
        return signed * sign;
    }

    /** Smallest |demand| whose pulse is strictly outside the measured deadband. */
    public double minMotionDemand() {
        if (!deadbandMeasured()) return Double.NaN;
        double beyondHigh = demandForPulse(Math.min(highUs, deadbandHighUs + 1));
        double beyondLow = demandForPulse(Math.max(lowUs, deadbandLowUs - 1));
        if (!Double.isFinite(beyondHigh) || !Double.isFinite(beyondLow)) return Double.NaN;
        double min = Math.max(Math.abs(beyondHigh), Math.abs(beyondLow));
        return min > 0 && min <= 1 ? min : Double.NaN;
    }

    /**
     * Sign-test magnitude: at least the requested value and just outside the
     * measured deadband, never above the bench cap. NaN if that is impossible.
     */
    public double validatedSignTestPower(double requested, double cap) {
        double min = minMotionDemand();
        if (!Double.isFinite(min) || !Double.isFinite(requested) || !Double.isFinite(cap)
                || requested <= 0 || cap <= 0 || cap > 1) {
            return Double.NaN;
        }
        double power = Math.max(requested, min);
        if (power > cap) return Double.NaN;
        double pulse = pulseUs(power);
        if (pulse >= deadbandLowUs && pulse <= deadbandHighUs) return Double.NaN;
        return power;
    }
}
