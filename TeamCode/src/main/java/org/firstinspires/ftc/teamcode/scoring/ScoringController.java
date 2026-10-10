package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Nonblocking sensed shot cycle. Outputs are requests; final hardware gates still apply. */
public final class ScoringController {
    public enum State { IDLE, PREPARE, FEED_ONE, CONFIRM_EXIT, REVERSE, WAIT_FOR_HIVE, FAULT }
    public static final class Input {
        public double nowSec,acquiredSec,rpm;
        public double hiveObservedSec=Double.NaN,targetObservedSec=Double.NaN;
        public double rangeInches=Double.NaN;
        public boolean healthy,enabled,start,cancel,resetFault,staged,exit,aimReady,targetFresh,stationary,hiveStable;
    }
    public static final class Output {
        public final double rpm,hood,compression,transfer,feeder;
        public final String reason;
        Output(double rpm,double hood,double compression,double transfer,double feeder,String reason) {
            this.rpm=rpm;this.hood=hood;this.compression=compression;this.transfer=transfer;this.feeder=feeder;this.reason=reason;
        }
        public static Output stopped(String reason) { return new Output(0,Double.NaN,Double.NaN,0,0,reason); }
    }
    public static final class Timing {
        public final double prepare,feed,exit,reverse,hive,maxAge;
        public final int retries;
        public Timing(double prepare,double feed,double exit,double reverse,double hive,double maxAge,int retries) {
            for (double n:new double[]{prepare,feed,exit,reverse,hive,maxAge}) if (!Double.isFinite(n)||n<=0) throw new IllegalArgumentException("Measured positive timing required");
            if (retries<0||retries>3) throw new IllegalArgumentException("Bounded retries required");
            this.prepare=prepare;this.feed=feed;this.exit=exit;this.reverse=reverse;this.hive=hive;this.maxAge=maxAge;this.retries=retries;
        }
    }
    private final PieceInventory inventory;
    private final Map<BallType,ShotRecipe> recipes;
    private final Timing timing;
    private final double forwardPower,reversePower;
    private State state=State.IDLE;
    private double entered,lastNow=Double.NaN,rpmSince=Double.NaN;
    private boolean previousStart,previousReset,previousExit,initialized;
    private ShotRecipe recipe;
    private int retries;
    private String fault="";
    public ScoringController(PieceInventory inventory,List<ShotRecipe> recipes,Timing timing) {
        this(inventory,recipes,timing,.25,.25);
    }
    /** The convenience constructor uses simulation powers. Hardware supplies bench measured powers. */
    public ScoringController(PieceInventory inventory,List<ShotRecipe> recipes,Timing timing,double forwardPower,double reversePower) {
        if (inventory==null||timing==null) throw new IllegalArgumentException("Inventory and timing required");
        for(double p:new double[]{forwardPower,reversePower})if(!Double.isFinite(p)||p<=0||p>1)throw new IllegalArgumentException("Measured mechanism powers required");
        this.forwardPower=forwardPower;this.reversePower=reversePower;
        this.inventory=inventory;this.timing=timing;this.recipes=new EnumMap<BallType,ShotRecipe>(BallType.class);
        if (recipes!=null) for (ShotRecipe r:recipes) { if (r==null) throw new IllegalArgumentException("Null recipe");this.recipes.put(r.type,r); }
    }
    public State state() { return state; }
    public String fault() { return fault; }
    public void cancel() {
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE) {
            inventory.markUncertain();fault="RELEASE CANCELLED; RECONCILE INVENTORY";state=State.FAULT;
        }else if(state!=State.FAULT)state=State.IDLE;
        recipe=null;rpmSince=Double.NaN;
    }
    private void resetFault() { fault="";state=State.IDLE;cancel(); }
    private void transition(State next,double now) { state=next;entered=now; }
    private Output fail(String message,double now) {
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE)inventory.markUncertain();
        fault=message;transition(State.FAULT,now);return Output.stopped(message);
    }
    public Output update(Input in) {
        boolean startEdge=in.start&&!previousStart,resetEdge=in.resetFault&&!previousReset;
        boolean exitEdge=initialized && in.exit&&!previousExit;
        previousStart=in.start;previousReset=in.resetFault;previousExit=in.exit;initialized=true;
        boolean fresh=Double.isFinite(in.nowSec)&&Double.isFinite(in.acquiredSec)
                && in.nowSec>=in.acquiredSec&&in.nowSec-in.acquiredSec<=timing.maxAge;
        boolean clock=Double.isFinite(in.nowSec)&&(!Double.isFinite(lastNow)||in.nowSec>=lastNow);
        lastNow=in.nowSec;
        if (!clock||!fresh||!in.healthy) return fail("SENSOR / CLOCK FAULT",in.nowSec);
        if (in.cancel||!in.enabled) {
            if(state!=State.FAULT)cancel();
            if(state==State.FAULT&&resetEdge&&!in.start&&!in.exit&&!in.enabled&&inventory.isConfirmed()) resetFault();
            return Output.stopped("DISABLED / CANCELLED");
        }
        if (state==State.FAULT) {
            return Output.stopped(fault.isEmpty()?"REARM REQUIRED":fault);
        }
        if (exitEdge && (state==State.FEED_ONE||state==State.CONFIRM_EXIT)) {
            if (!inventory.confirmExit(recipe.type)) return fail("INVENTORY EXIT MISMATCH",in.nowSec);
            transition(State.WAIT_FOR_HIVE,in.nowSec);rpmSince=Double.NaN;
            return Output.stopped("WAIT FOR HIVE / REACQUIRE");
        }
        if(exitEdge&&state!=State.WAIT_FOR_HIVE){inventory.markUncertain();return fail("UNEXPECTED EXIT; RECONCILE INVENTORY",in.nowSec);}
        if (state==State.IDLE && startEdge) {
            recipe=recipes.get(inventory.first());
            if (recipe==null) return Output.stopped("CONFIRMED PIECE / MEASURED RECIPE REQUIRED");
            if (in.exit) return fail("EXIT SENSOR BLOCKED",in.nowSec);
            retries=0;rpmSince=Double.NaN;transition(State.PREPARE,in.nowSec);
        }
        if (state==State.IDLE) return Output.stopped("IDLE");
        double elapsed=in.nowSec-entered;
        if (state==State.WAIT_FOR_HIVE) {
            // Never reuse the pre-release HIVE/aim readiness.
            if (in.hiveStable && in.targetFresh && observedFresh(in.nowSec,in.hiveObservedSec) && observedFresh(in.nowSec,in.targetObservedSec)
                    && in.hiveObservedSec>entered && in.targetObservedSec>entered
                    && elapsed>=recipe.settleSec) {
                cancel();return Output.stopped("SHOT CONFIRMED; NEW START REQUIRED");
            }
            if (elapsed>timing.hive) return fail("HIVE REACQUISITION TIMEOUT",in.nowSec);
            return Output.stopped("WAIT FOR HIVE / REACQUIRE");
        }
        if (state==State.REVERSE) {
            if (elapsed>=timing.reverse) { rpmSince=Double.NaN;transition(State.PREPARE,in.nowSec); }
            else return new Output(0,Double.NaN,Double.NaN,-reversePower,-reversePower,"BOUNDED JAM RECOVERY");
        }
        elapsed=in.nowSec-entered;
        boolean range=!recipe.rangeCalibrated||(Double.isFinite(in.rangeInches)&&in.rangeInches>=recipe.minRange&&in.rangeInches<=recipe.maxRange);
        boolean aim=in.aimReady&&in.targetFresh&&in.stationary&&in.hiveStable&&range
                &&observedFresh(in.nowSec,in.hiveObservedSec)&&observedFresh(in.nowSec,in.targetObservedSec);
        boolean rpmOk=Double.isFinite(in.rpm)&&Math.abs(in.rpm-recipe.rpm)<=recipe.rpmTolerance;
        if (!rpmOk) rpmSince=Double.NaN;
        else if (!Double.isFinite(rpmSince)) rpmSince=in.nowSec;
        if (state==State.PREPARE) {
            if (in.nowSec-entered>timing.prepare) return retryOrFault(in,"STAGING / READY TIMEOUT");
            boolean ready=aim&&in.staged&&elapsed>=recipe.settleSec
                    && Double.isFinite(rpmSince)&&in.nowSec-rpmSince>=recipe.rpmStableSec;
            if (ready) transition(State.FEED_ONE,in.nowSec);
            else return new Output(recipe.rpm,recipe.hood,recipe.compression,in.staged?0:forwardPower,0,
                    !in.staged?"STAGE PIECE":!aim?"AIM / HIVE / MOTION INHIBIT":"SPIN / SERVO SETTLE");
        }
        if (!aim || !rpmOk) {
            // A partly released piece is ambiguous. Never silently resume/retry it.
            return fail("READINESS LOST DURING RELEASE",in.nowSec);
        }
        if (state==State.FEED_ONE) {
            if (in.nowSec-entered>=timing.feed) transition(State.CONFIRM_EXIT,in.nowSec);
            else return new Output(recipe.rpm,recipe.hood,recipe.compression,forwardPower,forwardPower,"FEED ONE");
        }
        if (state==State.CONFIRM_EXIT) {
            if (in.nowSec-entered>timing.exit) return retryOrFault(in,"EXIT CONFIRMATION TIMEOUT");
            return new Output(recipe.rpm,recipe.hood,recipe.compression,0,0,"CONFIRM EXIT");
        }
        return Output.stopped("IDLE");
    }
    private Output retryOrFault(Input in,String message) {
        if (in.staged && !in.exit && retries<timing.retries) {
            retries++;transition(State.REVERSE,in.nowSec);
            return Output.stopped("STOP BEFORE RECOVERY");
        }
        return fail(message,in.nowSec);
    }
    private boolean observedFresh(double now,double at) {return Double.isFinite(at)&&at<=now&&now-at<=timing.maxAge;}
}
