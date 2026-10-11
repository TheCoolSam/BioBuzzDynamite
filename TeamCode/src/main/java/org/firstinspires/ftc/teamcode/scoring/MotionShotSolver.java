package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** Empirical flight-time lead, including chassis/pivot/muzzle motion at release.
 * The calibrated equivalent range is |target - releasePosition - releaseVelocity * flightTime|.
 * Flight-time tables characterize the actual hood, RPM, ball and target height together.
 */
public final class MotionShotSolver {
    public static final class Settings {
        public final double releaseDelaySec, releaseUncertaintySec, muzzleForward, muzzleLeft;
        public final double targetUncertaintyInches, calibrationErrorInches, maxMissInches;
        public final double maxTurretRate,maxTurretAcceleration,turretRateUncertainty;
        public final MotionConfidence.Limits motion;
        public Settings(double releaseDelaySec,double releaseUncertaintySec,double muzzleForward,double muzzleLeft,
                double targetUncertaintyInches,double calibrationErrorInches,double maxMissInches,double maxTurretRate,
                double maxTurretAcceleration,double turretRateUncertainty,MotionConfidence.Limits motion) {
            for(double n:new double[]{releaseDelaySec,releaseUncertaintySec,targetUncertaintyInches,calibrationErrorInches,maxMissInches,
                    maxTurretRate,maxTurretAcceleration,turretRateUncertainty})
                if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured release/error bounds required");
            if(!Double.isFinite(muzzleForward)||!Double.isFinite(muzzleLeft)||motion==null
                    ||releaseUncertaintySec>=releaseDelaySec)throw new IllegalArgumentException("Measured muzzle and motion required");
            this.releaseDelaySec=releaseDelaySec;this.releaseUncertaintySec=releaseUncertaintySec;
            this.muzzleForward=muzzleForward;this.muzzleLeft=muzzleLeft;
            this.targetUncertaintyInches=targetUncertaintyInches;this.maxMissInches=maxMissInches;this.motion=motion;
            this.calibrationErrorInches=calibrationErrorInches;
            this.maxTurretRate=maxTurretRate;this.maxTurretAcceleration=maxTurretAcceleration;
            this.turretRateUncertainty=turretRateUncertainty;
        }
    }
    public static final class Solution {
        public final ShotSelector.Selection selection;
        public final double releaseSec,releaseX,releaseY,releaseVx,releaseVy,releaseHeading;
        public final double bearing,equivalentRange,errorBoundInches;
        public final boolean confident;
        private Solution(ShotSelector.Selection selection,double releaseSec,double x,double y,double vx,double vy,
                double heading,double bearing,double range,double error,boolean confident) {
            this.selection=selection;this.releaseSec=releaseSec;releaseX=x;releaseY=y;releaseVx=vx;releaseVy=vy;
            releaseHeading=heading;this.bearing=bearing;equivalentRange=range;errorBoundInches=error;this.confident=confident;
        }
    }
    public final Settings settings;
    public MotionShotSolver(Settings settings) {if(settings==null)throw new IllegalArgumentException("Solver settings required");this.settings=settings;}
    public Solution solve(ShotSelector selector,BallType type,RobotState r,double now,double releaseSec,
            double ax,double ay,double alpha,double pivotForward,double pivotLeft,
            double turretAngle,double turretRate,double targetX,double targetY,boolean motionTrusted) {
        if(selector==null||r==null||!r.isFresh(now,settings.motion.maxAge)||!r.isVelocityValid())return null;
        for(double n:new double[]{now,releaseSec,ax,ay,alpha,pivotForward,pivotLeft,turretAngle,turretRate,targetX,targetY})
            if(!Double.isFinite(n))return null;
        double delay=releaseSec-now, prediction=releaseSec-r.getTimestampSec();
        if(delay<0||delay>settings.releaseDelaySec+1e-8||prediction<0||Math.abs(turretRate)>settings.maxTurretRate)return null;
        double h=r.getHeadingRad()+r.getAngularVelocityRadPerSec()*prediction+.5*alpha*prediction*prediction;
        double w=r.getAngularVelocityRadPerSec()+alpha*prediction;
        double phx=Math.cos(h)*pivotForward-Math.sin(h)*pivotLeft;
        double phy=Math.sin(h)*pivotForward+Math.cos(h)*pivotLeft;
        double muzzleYaw=h+turretAngle+turretRate*delay;
        double mx=Math.cos(muzzleYaw)*settings.muzzleForward-Math.sin(muzzleYaw)*settings.muzzleLeft;
        double my=Math.sin(muzzleYaw)*settings.muzzleForward+Math.cos(muzzleYaw)*settings.muzzleLeft;
        double x=r.getFieldX()+r.getFieldVx()*prediction+.5*ax*prediction*prediction+phx+mx;
        double y=r.getFieldY()+r.getFieldVy()*prediction+.5*ay*prediction*prediction+phy+my;
        double vx=r.getFieldVx()+ax*prediction-w*phy-(w+turretRate)*my;
        double vy=r.getFieldVy()+ay*prediction+w*phx+(w+turretRate)*mx;
        double range=Math.hypot(targetX-x,targetY-y),dx=0,dy=0;
        ShotSelector.Selection selected=null;
        boolean converged=false;
        for(int i=0;i<24;i++) {
            selected=selector.select(type,range);
            if(selected==null||!selected.hasFlightMeasurement())return null;
            dx=targetX-x-vx*selected.flightSec;dy=targetY-y-vy*selected.flightSec;
            double next=Math.hypot(dx,dy);
            if(Math.abs(next-range)<.001) {converged=true;break;}
            range=next;
        }
        if(!converged||range<=0)return null;
        double horizon=prediction+settings.releaseUncertaintySec;
        double uncertaintyAge=Math.max(0,now-r.getAcquisitionTimestampSec());
        double error=settings.motion.positionUncertainty+settings.targetUncertaintyInches+settings.calibrationErrorInches
                +settings.motion.velocityUncertainty*(horizon+selected.flightSec+uncertaintyAge)
                +settings.motion.maxAcceleration*(.5*horizon*horizon+horizon*selected.flightSec)
                +Math.hypot(vx,vy)*(selected.flightUncertaintySec+settings.releaseUncertaintySec)
                +range*(settings.motion.headingUncertainty+settings.motion.omegaUncertainty*horizon
                    +.5*settings.motion.maxAlpha*horizon*horizon+Math.abs(w+turretRate)*settings.releaseUncertaintySec)
                +(Math.hypot(pivotForward,pivotLeft)+Math.hypot(mx,my))
                    *(settings.motion.maxAlpha*(.5*horizon*horizon+horizon*selected.flightSec)
                        +settings.motion.omegaUncertainty*selected.flightSec)
                +range*(settings.turretRateUncertainty*horizon+.5*settings.maxTurretAcceleration*horizon*horizon)
                +Math.hypot(mx,my)*(settings.turretRateUncertainty+settings.maxTurretAcceleration*horizon)*selected.flightSec;
        if(!Double.isFinite(error)||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(vx)||!Double.isFinite(vy))return null;
        return new Solution(selected,releaseSec,x,y,vx,vy,AngleUtil.wrapRadians(h),Math.atan2(dy,dx),range,error,
                motionTrusted&&error<=settings.maxMissInches);
    }
}
