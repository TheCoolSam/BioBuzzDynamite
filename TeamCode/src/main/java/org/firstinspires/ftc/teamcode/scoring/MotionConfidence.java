package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** Detects pose/rate disagreement and pushes using distinct localizer acquisitions.
 * Bounds must include undetectable common-mode slip; these residuals are not a slip sensor.
 */
public final class MotionConfidence {
    public static final class Limits {
        public final double maxAge, maxSpeed, maxOmega, maxAcceleration, maxAlpha;
        public final double positionResidual, headingResidual, recoverySec;
        public final double positionUncertainty, velocityUncertainty, headingUncertainty;
        public final double omegaUncertainty;
        public Limits(double maxAge,double maxSpeed,double maxOmega,double maxAcceleration,double maxAlpha,
                double positionResidual,double headingResidual,double recoverySec,
                double positionUncertainty,double velocityUncertainty,double headingUncertainty,double omegaUncertainty) {
            for(double n:new double[]{maxAge,maxSpeed,maxOmega,maxAcceleration,maxAlpha,positionResidual,
                    headingResidual,recoverySec,positionUncertainty,velocityUncertainty,headingUncertainty,omegaUncertainty})
                if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured motion bounds required");
            this.maxAge=maxAge;this.maxSpeed=maxSpeed;this.maxOmega=maxOmega;
            this.maxAcceleration=maxAcceleration;this.maxAlpha=maxAlpha;
            this.positionResidual=positionResidual;this.headingResidual=headingResidual;this.recoverySec=recoverySec;
            this.positionUncertainty=positionUncertainty;this.velocityUncertainty=velocityUncertainty;
            this.headingUncertainty=headingUncertainty;
            this.omegaUncertainty=omegaUncertainty;
        }
    }
    public final Limits limits;
    private RobotState previous;
    private double trustedSince=Double.NaN,lastNow=Double.NaN;
    public double ax,ay,alpha;
    public String reason="MOTION WARMUP";
    public MotionConfidence(Limits limits) {if(limits==null)throw new IllegalArgumentException("Motion limits required");this.limits=limits;}
    public void reset() {previous=null;trustedSince=lastNow=Double.NaN;ax=ay=alpha=0;reason="MOTION WARMUP";}
    public boolean update(RobotState r,double now) {
        boolean clock=Double.isFinite(now)&&(!Double.isFinite(lastNow)||now>=lastNow);lastNow=now;
        if(!clock||r==null||!r.isFresh(now,limits.maxAge)||!r.isVelocityValid())return reject("MOTION STALE / INVALID");
        if(Math.hypot(r.getFieldVx(),r.getFieldVy())>limits.maxSpeed
                ||Math.abs(r.getAngularVelocityRadPerSec())>limits.maxOmega)return reject("MOTION OUTSIDE CALIBRATION");
        if(previous==null||previous.getResetGeneration()!=r.getResetGeneration()) {
            previous=r;trustedSince=r.getAcquisitionTimestampSec();reason="MOTION WARMUP";return false;
        }
        double dt=r.getTimestampSec()-previous.getTimestampSec();
        double acquisitionDt=r.getAcquisitionTimestampSec()-previous.getAcquisitionTimestampSec();
        if(dt<=0||acquisitionDt<=0) {reason="DISTINCT MOTION SAMPLE REQUIRED";return false;}
        ax=(r.getFieldVx()-previous.getFieldVx())/dt;
        ay=(r.getFieldVy()-previous.getFieldVy())/dt;
        alpha=(r.getAngularVelocityRadPerSec()-previous.getAngularVelocityRadPerSec())/dt;
        double residual=Math.hypot(r.getFieldX()-previous.getFieldX()-.5*(r.getFieldVx()+previous.getFieldVx())*dt,
                r.getFieldY()-previous.getFieldY()-.5*(r.getFieldVy()+previous.getFieldVy())*dt);
        double yawResidual=Math.abs(AngleUtil.wrapRadians(r.getHeadingRad()-previous.getHeadingRad()
                -.5*(r.getAngularVelocityRadPerSec()+previous.getAngularVelocityRadPerSec())*dt));
        previous=r;
        if(dt>limits.maxAge||acquisitionDt>limits.maxAge||Math.hypot(ax,ay)>limits.maxAcceleration
                ||Math.abs(alpha)>limits.maxAlpha||residual>limits.positionResidual||yawResidual>limits.headingResidual) {
            trustedSince=r.getAcquisitionTimestampSec();reason="PUSH / SLIP / MOTION DISAGREEMENT";return false;
        }
        if(!Double.isFinite(trustedSince))trustedSince=r.getAcquisitionTimestampSec();
        boolean ready=r.getAcquisitionTimestampSec()-trustedSince>=limits.recoverySec;
        reason=ready?"MOTION WITHIN BOUNDS":"MOTION RECOVERY";return ready;
    }
    private boolean reject(String message) {previous=null;trustedSince=Double.NaN;ax=ay=alpha=0;reason=message;return false;}
}
