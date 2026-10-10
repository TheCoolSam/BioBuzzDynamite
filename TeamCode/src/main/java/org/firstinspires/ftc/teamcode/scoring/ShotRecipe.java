package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** One measured ball-specific stationary shot recipe. Servo positions are normalized. */
public final class ShotRecipe {
    public final BallType type;
    public final double rpm, hood, compression, settleSec, rpmTolerance, rpmStableSec;
    public final double minRange,maxRange;
    public final boolean rangeCalibrated;
    public ShotRecipe(BallType type,double rpm,double hood,double compression,double settleSec,
            double rpmTolerance,double rpmStableSec) {
        this(type,rpm,hood,compression,settleSec,rpmTolerance,rpmStableSec,0,Double.POSITIVE_INFINITY,false);
    }
    public ShotRecipe(BallType type,double rpm,double hood,double compression,double settleSec,
            double rpmTolerance,double rpmStableSec,double minRange,double maxRange) {
        this(type,rpm,hood,compression,settleSec,rpmTolerance,rpmStableSec,minRange,maxRange,true);
    }
    private ShotRecipe(BallType type,double rpm,double hood,double compression,double settleSec,
            double rpmTolerance,double rpmStableSec,double minRange,double maxRange,boolean calibrated) {
        if (type==null || !Double.isFinite(rpm) || rpm<=0 || !unit(hood) || !unit(compression)
                || !Double.isFinite(settleSec) || settleSec<=0 || !Double.isFinite(rpmTolerance)
                || rpmTolerance<=0 || !Double.isFinite(rpmStableSec) || rpmStableSec<=0) {
            throw new IllegalArgumentException("Measured shot recipe required");
        }
        this.type=type;this.rpm=rpm;this.hood=hood;this.compression=compression;
        this.settleSec=settleSec;this.rpmTolerance=rpmTolerance;this.rpmStableSec=rpmStableSec;
        if(calibrated&&(!Double.isFinite(minRange)||!Double.isFinite(maxRange)||minRange<=0||maxRange<=minRange))throw new IllegalArgumentException("Measured shot range required");
        this.minRange=minRange;this.maxRange=maxRange;this.rangeCalibrated=calibrated;
    }
    private static boolean unit(double v) { return Double.isFinite(v) && v>=0 && v<=1; }
}
