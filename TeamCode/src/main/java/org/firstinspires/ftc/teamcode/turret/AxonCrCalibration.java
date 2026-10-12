package org.firstinspires.ftc.teamcode.turret;

/** Measured SDK power-to-pulse calibration. Zero demand always writes the verified neutral. */
public final class AxonCrCalibration {
    public final double lowUs, neutralUs, highUs, frameUs;
    public final int sign;
    public AxonCrCalibration(double lowUs,double neutralUs,double highUs,double frameUs,int sign) {
        if(!Double.isFinite(lowUs)||!Double.isFinite(neutralUs)||!Double.isFinite(highUs)
                ||!Double.isFinite(frameUs)||lowUs<=0||neutralUs<=lowUs||highUs<=neutralUs
                ||frameUs<=highUs||(sign!=1&&sign!=-1))
            throw new IllegalArgumentException("Measured CR endpoints, neutral, frame and sign required");
        this.lowUs=lowUs;this.neutralUs=neutralUs;this.highUs=highUs;this.frameUs=frameUs;this.sign=sign;
    }
    public double sdkPower(double demand) {
        if(!Double.isFinite(demand)||Math.abs(demand)>1)throw new IllegalArgumentException("Bounded CR demand required");
        double signed=demand*sign;
        double pulse=neutralUs+signed*(signed>=0?highUs-neutralUs:neutralUs-lowUs);
        return 2*(pulse-lowUs)/(highUs-lowUs)-1;
    }
}
