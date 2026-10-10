package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.state.RobotState;

/** One deliberate pickup or scoring-position move. Completion requires a sensed entry.
 * Releasing the hold or moving the sticks cancels in the OpMode. A new press replans.
 */
public final class PickupAssist {
    public static final class Settings {
        public final double positionTolerance,headingTolerance,kp,turnKp,cap,timeout;
        public final SweptCaptureFeasibility footprint;
        public Settings(double positionTolerance,double headingTolerance,double kp,double turnKp,
                double cap,double timeout,SweptCaptureFeasibility footprint) {
            for(double n:new double[]{positionTolerance,headingTolerance,kp,turnKp,cap,timeout})
                if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured pickup motion settings required");
            if(cap>1||footprint==null)throw new IllegalArgumentException("Pickup cap and measured footprint required");
            this.positionTolerance=positionTolerance;this.headingTolerance=headingTolerance;this.kp=kp;
            this.turnKp=turnKp;this.cap=cap;this.timeout=timeout;this.footprint=footprint;
        }
    }
    public static final class Command {
        public final double x,y,turn;
        public final boolean intake,done,fault;
        public final String reason;
        Command(double x,double y,double turn,boolean intake,boolean done,boolean fault,String reason) {
            this.x=x;this.y=y;this.turn=turn;this.intake=intake;this.done=done;this.fault=fault;this.reason=reason;
        }
    }
    private final Settings settings;
    private final double[] xs,ys,headings;
    private final long generation;
    private final int initialCount;
    public final BallType expectedType;
    private int index;
    private double legStarted,lastNow;
    private boolean completed,failed,pickupSeen;
    public PickupAssist(Settings settings,PickupPlan plan,RobotState robot,double now,int confirmedCount) {
        if(settings==null||plan==null||robot==null||!robot.isFresh(now,.2))throw new IllegalArgumentException("Fresh executable pickup plan required");
        this.settings=settings;generation=robot.getResetGeneration();initialCount=confirmedCount;legStarted=lastNow=now;
        if(plan.getDecision()==PickupPlan.Decision.PICKUP&&!plan.getTargets().isEmpty()) {
            PickupTarget t=plan.getTargets().get(0);expectedType=t.getPiece().getType();
            xs=new double[]{t.getApproachX(),t.getCaptureX()};ys=new double[]{t.getApproachY(),t.getCaptureY()};
            headings=new double[]{t.getApproachHeadingRad(),t.getApproachHeadingRad()};
        }else if(plan.getDecision()==PickupPlan.Decision.SHOOT_NOW&&plan.getShotSetup()!=null) {
            ShotSetupPlan s=plan.getShotSetup();expectedType=null;
            xs=new double[]{s.x};ys=new double[]{s.y};headings=new double[]{s.headingRad};
        }else throw new IllegalArgumentException("Plan has no executable pickup/scoring pose");
        double x=robot.getFieldX(),y=robot.getFieldY();
        for(int i=0;i<xs.length;i++) {
            if(!settings.footprint.segmentSafe(x,y,xs[i],ys[i]))throw new IllegalArgumentException("Pickup route blocked");
            x=xs[i];y=ys[i];
        }
    }
    public Command update(RobotState robot,double now,int confirmedCount) {
        if(failed)return zero(false,true,"PICKUP FAULT; NEW PRESS REQUIRED");
        if(completed)return zero(true,false,"POSITION / PICKUP CONFIRMED");
        if(robot==null||!robot.isFresh(now,.2)||robot.getResetGeneration()!=generation||now<lastNow
                ||now-legStarted>settings.timeout||confirmedCount<initialCount||confirmedCount>initialCount+1) {
            failed=true;return zero(false,true,"PICKUP POSE / TIME / INVENTORY FAULT");
        }
        lastNow=now;
        if(expectedType!=null&&confirmedCount==initialCount+1)pickupSeen=true;
        if(!settings.footprint.segmentSafe(robot.getFieldX(),robot.getFieldY(),xs[index],ys[index])) {
            failed=true;return zero(false,true,"PICKUP CORRIDOR BLOCKED");
        }
        double dx=xs[index]-robot.getFieldX(),dy=ys[index]-robot.getFieldY();
        double dh=AngleUtil.wrapRadians(headings[index]-robot.getHeadingRad());
        boolean intake=expectedType!=null&&index==xs.length-1&&!pickupSeen;
        if(Math.hypot(dx,dy)<=settings.positionTolerance&&Math.abs(dh)<=settings.headingTolerance) {
            if(index==xs.length-1) {
                if(expectedType==null||pickupSeen) {completed=true;return zero(true,false,"POSITION / PICKUP CONFIRMED");}
                return new Command(0,0,0,true,false,false,"WAIT FOR CONFIRMED ENTRY");
            }
            index++;legStarted=now;return zero(false,false,"BEGIN CAPTURE SEGMENT");
        }
        double scale=Math.max(1,Math.hypot(dx*settings.kp,dy*settings.kp)/settings.cap);
        return new Command(dx*settings.kp/scale,dy*settings.kp/scale,
                AngleUtil.clamp(dh*settings.turnKp,-settings.cap,settings.cap),intake,false,false,"PICKUP / SCORING POSITION");
    }
    private static Command zero(boolean done,boolean fault,String reason) {return new Command(0,0,0,false,done,fault,reason);}
}
