package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** Empirical flight-time lead, including chassis/pivot/muzzle motion at release.
 * The calibrated equivalent range is |target - releasePosition - releaseVelocity * flightTime|.
 * Flight-time tables characterize the actual hood, RPM, ball and target height together.
 */
public final class MotionShotSolver {
    /** Stationary characterization; independent of moving-shot acceleration/lead tuning. */
    public static final class StationarySettings {
        public final double positionError,targetError,shotError,maxMiss,stableSec;
        public StationarySettings(double positionError,double targetError,double shotError,double maxMiss,double stableSec) {
            for(double n:new double[]{positionError,targetError,shotError,maxMiss,stableSec})
                if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured stationary error/stability bounds required");
            this.positionError=positionError;this.targetError=targetError;this.shotError=shotError;this.maxMiss=maxMiss;this.stableSec=stableSec;
        }
    }
    public static Solution stationary(ShotSelector selector,BallType type,RobotState r,double now,
            double pivotForward,double pivotLeft,double yaw,double yawRate,double tx,double ty,double tz,
            double maxSpeed,double maxRate,StationarySettings settings) {
        if(selector==null||r==null||settings==null||!r.isFresh(now,.2)||!r.isVelocityValid())return null;
        for(double n:new double[]{pivotForward,pivotLeft,yaw,yawRate,tx,ty,tz,maxSpeed,maxRate})if(!Double.isFinite(n))return null;
        if(maxSpeed<=0||maxRate<=0||Math.hypot(r.getFieldVx(),r.getFieldVy())>maxSpeed
                ||Math.abs(r.getAngularVelocityRadPerSec())>maxRate||Math.abs(yawRate)>maxRate)return null;
        double h=r.getHeadingRad(),a=h+yaw;
        double px=r.getFieldX()+Math.cos(h)*pivotForward-Math.sin(h)*pivotLeft;
        double py=r.getFieldY()+Math.sin(h)*pivotForward+Math.cos(h)*pivotLeft;
        double range=Math.hypot(tx-px,ty-py);
        // Seed from a real geometry knot so a valid muzzle range is not rejected merely
        // because the initial pivot range falls beyond the table's outer endpoint.
        if(selector instanceof CalibratedShotTable) {
            CalibratedShotTable.Point seed=null;
            for(CalibratedShotTable.Point point:((CalibratedShotTable)selector).points())
                if(point.recipe.type==type&&(seed==null||Math.abs(point.rangeInches-range)<Math.abs(seed.rangeInches-range)))seed=point;
            if(seed!=null&&seed.recipe.physical()) {
                LauncherGeometry.Release g=seed.recipe.geometry.at(seed.recipe.pitchRad,seed.recipe.gapMm);
                if(g!=null)range=Math.hypot(tx-px-Math.cos(a)*g.forward+Math.sin(a)*g.left,
                        ty-py-Math.sin(a)*g.forward-Math.cos(a)*g.left);
            }
        }
        for(int i=0;i<24;i++) {
            ShotSelector.Selection s=selector.select(type,range);
            if(s==null||!s.recipe.physical()||!s.hasFlightMeasurement()||!s.hasReleaseMeasurement()||!s.acceptsHeight(tz))return null;
            LauncherGeometry.Release g=s.recipe.geometry.at(s.recipe.pitchRad,s.recipe.gapMm);
            if(g==null)return null;
            double x=px+Math.cos(a)*g.forward-Math.sin(a)*g.left,y=py+Math.sin(a)*g.forward+Math.cos(a)*g.left;
            double next=Math.hypot(tx-x,ty-y);
            if(Math.abs(next-range)<.001&&next>0) {
                double release=s.releaseDelaySec+s.releaseUncertaintySec;
                double horizon=release+s.flightSec+s.flightUncertaintySec+Math.max(0,now-r.getAcquisitionTimestampSec());
                double lever=Math.hypot(pivotForward,pivotLeft)+Math.hypot(g.forward,g.left);
                double error=settings.positionError+settings.targetError+settings.shotError+s.recipe.geometry.errorInches
                        +maxSpeed*horizon+2*maxRate*lever*horizon+2*maxRate*next*release;
                return new Solution(s,now+s.releaseDelaySec,x,y,0,0,h,Math.atan2(ty-y,tx-x),range,error,
                        Double.isFinite(error)&&error<=settings.maxMiss,g.height,g.launchPitch);
            }
            range=next;
        }
        return null;
    }
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
        public final double releaseHeight,launchPitch;
        public Solution atRelease(double time) {
            if(!Double.isFinite(time))throw new IllegalArgumentException("Known release time required");
            return new Solution(selection,time,releaseX,releaseY,releaseVx,releaseVy,releaseHeading,bearing,equivalentRange,errorBoundInches,confident,releaseHeight,launchPitch);
        }
        private Solution(ShotSelector.Selection selection,double releaseSec,double x,double y,double vx,double vy,
                double heading,double bearing,double range,double error,boolean confident,double height,double launchPitch) {
            this.selection=selection;this.releaseSec=releaseSec;releaseX=x;releaseY=y;releaseVx=vx;releaseVy=vy;
            releaseHeading=heading;this.bearing=bearing;equivalentRange=range;errorBoundInches=error;this.confident=confident;releaseHeight=height;this.launchPitch=launchPitch;
        }
    }
    public final Settings settings;
    public MotionShotSolver(Settings settings) {if(settings==null)throw new IllegalArgumentException("Solver settings required");this.settings=settings;}
    public Solution solve(ShotSelector selector,BallType type,RobotState r,double now,double releaseSec,
            double ax,double ay,double alpha,double pivotForward,double pivotLeft,
            double turretAngle,double turretRate,double targetX,double targetY,boolean motionTrusted) {
        return solve(selector,type,r,now,releaseSec,ax,ay,alpha,pivotForward,pivotLeft,turretAngle,turretRate,
                targetX,targetY,Double.NaN,motionTrusted,true);
    }
    public Solution solve(ShotSelector selector,BallType type,RobotState r,double now,double releaseSec,
            double ax,double ay,double alpha,double pivotForward,double pivotLeft,
            double turretAngle,double turretRate,double targetX,double targetY,double targetHeight,
            boolean motionTrusted,boolean committedRelease) {
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
        LauncherGeometry.Release geometry=null;
        double geometryError=0;
        for(int i=0;i<24;i++) {
            selected=selector.select(type,range);
            if(selected==null||!selected.hasFlightMeasurement())return null;
            if(selected.recipe.physical()) {
                if(!selected.acceptsHeight(targetHeight)||!selected.hasReleaseMeasurement()
                        ||selected.releaseDelaySec>settings.releaseDelaySec||selected.releaseUncertaintySec>settings.releaseUncertaintySec)return null;
                geometry=selected.recipe.geometry.at(selected.recipe.pitchRad,selected.recipe.gapMm);
                if(geometry==null)return null;
                geometryError=selected.recipe.geometry.errorInches;
                if(!committedRelease) {
                    delay=selected.releaseDelaySec;releaseSec=now+delay;prediction=releaseSec-r.getTimestampSec();
                    if(delay>settings.releaseDelaySec||selected.releaseUncertaintySec>settings.releaseUncertaintySec)return null;
                    h=r.getHeadingRad()+r.getAngularVelocityRadPerSec()*prediction+.5*alpha*prediction*prediction;
                    w=r.getAngularVelocityRadPerSec()+alpha*prediction;
                    phx=Math.cos(h)*pivotForward-Math.sin(h)*pivotLeft;phy=Math.sin(h)*pivotForward+Math.cos(h)*pivotLeft;
                    muzzleYaw=h+turretAngle+turretRate*delay;
                }
                mx=Math.cos(muzzleYaw)*geometry.forward-Math.sin(muzzleYaw)*geometry.left;
                my=Math.sin(muzzleYaw)*geometry.forward+Math.cos(muzzleYaw)*geometry.left;
                x=r.getFieldX()+r.getFieldVx()*prediction+.5*ax*prediction*prediction+phx+mx;
                y=r.getFieldY()+r.getFieldVy()*prediction+.5*ay*prediction*prediction+phy+my;
                vx=r.getFieldVx()+ax*prediction-w*phy-(w+turretRate)*my;
                vy=r.getFieldVy()+ay*prediction+w*phx+(w+turretRate)*mx;
            }
            dx=targetX-x-vx*selected.flightSec;dy=targetY-y-vy*selected.flightSec;
            double next=Math.hypot(dx,dy);
            if(Math.abs(next-range)<.001) {converged=true;break;}
            range=next;
        }
        if(!converged||range<=0)return null;
        double horizon=prediction+settings.releaseUncertaintySec;
        double uncertaintyAge=Math.max(0,now-r.getAcquisitionTimestampSec());
        double error=settings.motion.positionUncertainty+settings.targetUncertaintyInches+settings.calibrationErrorInches+geometryError
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
                motionTrusted&&error<=settings.maxMissInches,geometry==null?Double.NaN:geometry.height,
                geometry==null?Double.NaN:geometry.launchPitch);
    }
}
