package org.firstinspires.ftc.teamcode.scoring;

/** Output-side measured state. Servo.getPosition() must never supply this sample. */
public final class LauncherFeedback {
    /** Conservative default for pitch/gap finite-difference smoothing. */
    public static final double DEFAULT_RATE_TAU_SEC = 0.05;
    public final double acquiredSec,pitchRad,pitchVelocity,gapMm,gapVelocity;
    public LauncherFeedback(double acquiredSec,double pitchRad,double pitchVelocity,double gapMm,double gapVelocity) {
        this.acquiredSec=acquiredSec;this.pitchRad=pitchRad;this.pitchVelocity=pitchVelocity;
        this.gapMm=gapMm;this.gapVelocity=gapVelocity;
    }
    public boolean fresh(double now,double maxAge) {
        return Double.isFinite(acquiredSec)&&acquiredSec<=now&&now-acquiredSec<=maxAge;
    }
    public static final class Limits {
        public final double pitchError,pitchRate,gapError,gapRate,stableSec,maxAge;
        public Limits(double pitchError,double pitchRate,double gapError,double gapRate,double stableSec,double maxAge) {
            for(double n:new double[]{pitchError,pitchRate,gapError,gapRate,stableSec,maxAge})
                if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured launcher readiness bounds required");
            this.pitchError=pitchError;this.pitchRate=pitchRate;this.gapError=gapError;this.gapRate=gapRate;
            this.stableSec=stableSec;this.maxAge=maxAge;
        }
        public boolean pitchReady(LauncherFeedback s,ShotRecipe r,double now) {
            return s!=null&&s.fresh(now,maxAge)&&Double.isFinite(s.pitchRad)&&Double.isFinite(s.pitchVelocity)
                    &&Math.abs(s.pitchRad-r.pitchRad)<=pitchError&&Math.abs(s.pitchVelocity)<=pitchRate;
        }
        public boolean compressionReady(LauncherFeedback s,ShotRecipe r,double now) {
            return s!=null&&s.fresh(now,maxAge)&&Double.isFinite(s.gapMm)&&Double.isFinite(s.gapVelocity)
                    &&Math.abs(s.gapMm-r.gapMm)<=gapError&&Math.abs(s.gapVelocity)<=gapRate;
        }
    }
}
