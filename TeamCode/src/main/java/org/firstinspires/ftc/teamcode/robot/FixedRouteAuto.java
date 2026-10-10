package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.planning.pickup.SweptCaptureFeasibility;

/** Fixed waypoint routine with bounded legs and an explicit park deadline. No route is invented. */
public final class FixedRouteAuto {
    public enum Action { DRIVE, SHOOT, PARK, DONE, FAULT }
    public static final class Waypoint {
        public final double x,y,headingRad,timeoutSec;
        public final boolean shoot;
        public final boolean collect;
        public Waypoint(double x,double y,double headingRad,double timeoutSec,boolean shoot) {
            this(x,y,headingRad,timeoutSec,shoot,false);
        }
        public Waypoint(double x,double y,double headingRad,double timeoutSec,boolean shoot,boolean collect) {
            if (!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(headingRad)||!Double.isFinite(timeoutSec)||timeoutSec<=0) throw new IllegalArgumentException("Measured waypoint required");
            this.x=x;this.y=y;this.headingRad=headingRad;this.timeoutSec=timeoutSec;this.shoot=shoot;
            if(shoot&&collect)throw new IllegalArgumentException("Separate pickup and scoring waypoints required");
            this.collect=collect;
        }
    }
    public static final class Command {
        public final Action action;
        public final double xPower,yPower,turnPower;
        public final String reason;
        Command(Action action,double x,double y,double turn,String reason) {this.action=action;xPower=x;yPower=y;turnPower=turn;this.reason=reason;}
    }
    private final Waypoint[] route;
    private final Waypoint park;
    private final SweptCaptureFeasibility bounds;
    private final double deadline,parkAt,positionTolerance,headingTolerance,kp,turnKp,cap;
    private int index;
    private boolean parking,waitingShot,finished,failed,pickupSeen;
    private long generation=Long.MIN_VALUE;
    private double start=Double.NaN,legStarted,lastNow=Double.NaN;
    public FixedRouteAuto(Waypoint[] route,Waypoint park,SweptCaptureFeasibility bounds,
            double deadline,double parkAt,double positionTolerance,double headingTolerance,double kp,double turnKp,double cap) {
        if (route==null||park==null||bounds==null) throw new IllegalArgumentException("Explicit route/park/footprint required");
        if(park.shoot||park.collect)throw new IllegalArgumentException("Park cannot release or collect pieces");
        for (Waypoint w:route) if (w==null) throw new IllegalArgumentException("Null waypoint");
        for (double n:new double[]{deadline,parkAt,positionTolerance,headingTolerance,kp,turnKp,cap}) if (!Double.isFinite(n)||n<=0) throw new IllegalArgumentException("Measured auto settings required");
        if (parkAt>=deadline||cap>1) throw new IllegalArgumentException("Park margin and legal cap required");
        this.route=route.clone();this.park=park;this.bounds=bounds;this.deadline=deadline;this.parkAt=parkAt;
        this.positionTolerance=positionTolerance;this.headingTolerance=headingTolerance;this.kp=kp;this.turnKp=turnKp;this.cap=cap;
    }
    public Command update(RobotState robot,double now,boolean shotConfirmed,boolean fault) {
        return update(robot,now,shotConfirmed,false,fault);
    }
    public boolean intakeRequested() {return !parking&&!finished&&!failed&&current().collect;}
    public Command update(RobotState robot,double now,boolean shotConfirmed,boolean pickupConfirmed,boolean fault) {
        if (failed) return zero(Action.FAULT,"LATCHED AUTO FAULT");
        if (finished) return zero(Action.DONE,"DONE");
        if (robot==null||!robot.isFresh(now,.2)) { failed=true;return zero(Action.FAULT,"POSE UNAVAILABLE"); }
        if (!Double.isFinite(start)) { start=legStarted=now;generation=robot.getResetGeneration();parking=route.length==0; }
        if ((Double.isFinite(lastNow)&&now<lastNow)||generation!=robot.getResetGeneration()) {failed=true;return zero(Action.FAULT,"CLOCK / LOCALIZATION RESET");}
        lastNow=now;
        if (now-start>=deadline) {finished=true;return zero(Action.DONE,"AUTO DEADLINE");}
        if (!parking && (fault||now-start>=parkAt||now-legStarted>current().timeoutSec)) {
            parking=true;waitingShot=false;pickupSeen=false;legStarted=now;
        }
        Waypoint target=current();
        if(target.collect&&pickupConfirmed)pickupSeen=true;
        if (parking&&now-legStarted>park.timeoutSec) {failed=true;return zero(Action.FAULT,"PARK TIMEOUT");}
        if (!bounds.segmentSafe(robot.getFieldX(),robot.getFieldY(),target.x,target.y)) {
            failed=true;return zero(Action.FAULT,"ROUTE / FOOTPRINT BLOCKED");
        }
        if (waitingShot) {
            if (!shotConfirmed) return zero(Action.SHOOT,"WAIT FOR CONFIRMED RELEASE");
            waitingShot=false;pickupSeen=false;index++;legStarted=now;
            if (index>=route.length) {parking=true;legStarted=now;}
            return zero(Action.DRIVE,"NEXT LEG");
        }
        double dx=target.x-robot.getFieldX(),dy=target.y-robot.getFieldY();
        double dh=AngleUtil.wrapRadians(target.headingRad-robot.getHeadingRad());
        if (Math.hypot(dx,dy)<=positionTolerance && Math.abs(dh)<=headingTolerance) {
            if (parking) {finished=true;return zero(Action.DONE,"PARKED");}
            if (target.shoot) {waitingShot=true;return zero(Action.SHOOT,"START SHOT");}
            if (target.collect&&!pickupSeen)return zero(Action.DRIVE,"WAIT FOR CONFIRMED PICKUP");
            pickupSeen=false;index++;legStarted=now;if(index>=route.length)parking=true;
            return zero(Action.DRIVE,"NEXT LEG");
        }
        double scale=Math.max(1,Math.hypot(dx*kp,dy*kp)/cap);
        return new Command(parking?Action.PARK:Action.DRIVE,dx*kp/scale,dy*kp/scale,
                AngleUtil.clamp(dh*turnKp,-cap,cap),parking?"PARK":"DRIVE");
    }
    private Waypoint current() {return parking||index>=route.length?park:route[index];}
    private static Command zero(Action action,String reason) {return new Command(action,0,0,0,reason);}
}
