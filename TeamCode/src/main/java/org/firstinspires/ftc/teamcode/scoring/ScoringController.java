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
        public ShotSelector.Selection selectedShot;
        public boolean motionReady;
        public boolean burst;
        public int targetId;
        public HiveTracker.State hiveState=HiveTracker.State.UNKNOWN;
        public double releaseDelaySec,releaseUncertaintySec;
        public double releaseSec=Double.NaN;
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
    private final ShotSelector selector;
    private final BurstPolicy burst;
    private final List<BurstPolicy.Flight> flights=new ArrayList<BurstPolicy.Flight>();
    private final Timing timing;
    private final double forwardPower,reversePower;
    private State state=State.IDLE;
    private double entered,lastNow=Double.NaN,rpmSince=Double.NaN;
    private boolean previousStart,previousReset,previousExit,initialized;
    private ShotRecipe recipe;
    private ShotSelector.Selection selection;
    private double setpointSince,lastExit=Double.NEGATIVE_INFINITY,reacquireAfter=Double.NEGATIVE_INFINITY;
    private double committedReleaseSec=Double.NaN,lastAcquired=Double.NaN;
    private double exitLatencySec=Double.NaN,impactSettleSec=Double.NaN,committedReleaseUncertainty;
    private boolean releasePredictionKnown;
    private boolean burstArmed;
    private double rpmRetargetFraction=.1,hoodRetargetTolerance=.001,compressionRetargetTolerance=.001;
    private int activeTarget;
    private int retries;
    private String fault="";
    public ScoringController(PieceInventory inventory,List<ShotRecipe> recipes,Timing timing) {
        this(inventory,recipes,timing,.25,.25);
    }
    /** The convenience constructor uses simulation powers. Hardware supplies bench measured powers. */
    public ScoringController(PieceInventory inventory,List<ShotRecipe> recipes,Timing timing,double forwardPower,double reversePower) {
        this(inventory,fixedSelector(recipes),timing,forwardPower,reversePower,null);
    }
    public ScoringController(PieceInventory inventory,ShotSelector selector,Timing timing,
            double forwardPower,double reversePower,BurstPolicy burst) {
        if (inventory==null||timing==null) throw new IllegalArgumentException("Inventory and timing required");
        if(selector==null)throw new IllegalArgumentException("Shot selector required");
        for(double p:new double[]{forwardPower,reversePower})if(!Double.isFinite(p)||p<=0||p>1)throw new IllegalArgumentException("Measured mechanism powers required");
        this.forwardPower=forwardPower;this.reversePower=reversePower;
        this.inventory=inventory;this.timing=timing;this.selector=selector;this.burst=burst;
    }
    private static ShotSelector fixedSelector(List<ShotRecipe> recipes) {
        final Map<BallType,ShotRecipe> fixed=new EnumMap<BallType,ShotRecipe>(BallType.class);
        if(recipes!=null)for(ShotRecipe r:recipes) {
            if(r==null||fixed.put(r.type,r)!=null)throw new IllegalArgumentException("Duplicate type requires a calibrated range table");
        }
        return (type,range)->{ShotRecipe r=type==null?null:fixed.get(type);
            return r==null?null:new ShotSelector.Selection(r,Double.NaN,Double.NaN);};
    }
    public ShotSelector selector() {return selector;}
    /** Bounds for debounced exit latency and post-impact HIVE motion, including single shots. */
    public void setFlightTiming(double measuredMaxLatencySec,double measuredImpactSettleSec) {
        if(state!=State.IDLE||!Double.isFinite(measuredMaxLatencySec)||measuredMaxLatencySec<=0
                ||!Double.isFinite(measuredImpactSettleSec)||measuredImpactSettleSec<=0)
            throw new IllegalArgumentException("Configure measured flight timing before arming");
        exitLatencySec=measuredMaxLatencySec;impactSettleSec=measuredImpactSettleSec;
    }
    /** Measured actuator/shot acceptance envelope. Sub-threshold changes hold the last command.
     * Calibration flight/error bounds must include this envelope, including during a release.
     */
    public void setRetargetTolerance(double rpmFraction,double hoodTolerance,double compressionTolerance) {
        if(state!=State.IDLE)throw new IllegalStateException("Configure tolerances before arming");
        for(double n:new double[]{rpmFraction,hoodTolerance,compressionTolerance})
            if(!Double.isFinite(n)||n<=0||n>1)throw new IllegalArgumentException("Measured retarget tolerances required");
        rpmRetargetFraction=rpmFraction;hoodRetargetTolerance=hoodTolerance;compressionRetargetTolerance=compressionTolerance;
    }
    public List<BurstPolicy.Flight> inFlight() {return Collections.unmodifiableList(new ArrayList<BurstPolicy.Flight>(flights));}
    public State state() { return state; }
    public String fault() { return fault; }
    public void cancel() {
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE) {
            retainPossibleFlight();
            inventory.markUncertain();fault="RELEASE CANCELLED; RECONCILE INVENTORY";state=State.FAULT;
        }else if(state!=State.FAULT)state=State.IDLE;
        recipe=null;selection=null;rpmSince=Double.NaN;burstArmed=false;
    }
    private void resetFault() { fault="";state=State.IDLE;cancel(); }
    private void transition(State next,double now) { state=next;entered=now; }
    private Output fail(String message,double now) {
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE) {retainPossibleFlight();inventory.markUncertain();}
        fault=message;transition(State.FAULT,now);return Output.stopped(message);
    }
    public Output update(Input in) {
        boolean startEdge=in.start&&!previousStart,resetEdge=in.resetFault&&!previousReset;
        boolean exitEdge=initialized && in.exit&&!previousExit;
        previousStart=in.start;previousReset=in.resetFault;previousExit=in.exit;initialized=true;
        boolean fresh=Double.isFinite(in.nowSec)&&Double.isFinite(in.acquiredSec)
                && in.nowSec>=in.acquiredSec&&in.nowSec-in.acquiredSec<=timing.maxAge
                &&(!Double.isFinite(lastAcquired)||in.acquiredSec>=lastAcquired);
        lastAcquired=in.acquiredSec;
        boolean clock=Double.isFinite(in.nowSec)&&(!Double.isFinite(lastNow)||in.nowSec>=lastNow);
        lastNow=in.nowSec;
        if (!clock||!fresh||!in.healthy) return fail("SENSOR / CLOCK FAULT",in.nowSec);
        retireFlights(in.nowSec);
        if (in.cancel||!in.enabled) {
            if(state!=State.FAULT)cancel();
            if(state==State.FAULT&&resetEdge&&!in.start&&!in.exit&&!in.enabled&&inventory.isConfirmed()) resetFault();
            return Output.stopped("DISABLED / CANCELLED");
        }
        if (state==State.FAULT) {
            return Output.stopped(fault.isEmpty()?"REARM REQUIRED":fault);
        }
        if(burstArmed&&!in.start&&state==State.PREPARE) {cancel();return Output.stopped("BURST TRIGGER RELEASED");}
        if (exitEdge && (state==State.FEED_ONE||state==State.CONFIRM_EXIT)) {
            if (!inventory.confirmExit(recipe.type)) return fail("INVENTORY EXIT MISMATCH",in.nowSec);
            lastExit=in.nowSec;
            if(selection.hasFlightMeasurement())flights.add(new BurstPolicy.Flight(recipe.type,activeTarget,in.nowSec,
                    in.nowSec-exitLatencySec,in.nowSec,selection.flightSec,selection.flightUncertaintySec,true));
            transition(State.WAIT_FOR_HIVE,in.nowSec);rpmSince=Double.NaN;
            if(releasePredictionKnown&&(committedReleaseSec+committedReleaseUncertainty<in.nowSec-exitLatencySec
                    ||committedReleaseSec-committedReleaseUncertainty>in.nowSec))
                return fail("RELEASE TIMING OUTSIDE CALIBRATION",in.nowSec);
            if(burstArmed)return holding("SHOT CONFIRMED / BURST RECOVERY");
            return Output.stopped("WAIT FOR HIVE / REACQUIRE");
        }
        if(exitEdge){inventory.markUncertain();return fail("UNEXPECTED EXIT; RECONCILE INVENTORY",in.nowSec);}
        if (state==State.IDLE && startEdge) {
            selection=select(in);recipe=selection==null?null:selection.recipe;
            if (recipe==null) return Output.stopped("CONFIRMED PIECE / CALIBRATED RANGE REQUIRED");
            burstArmed=burst!=null&&in.burst;
            if (in.exit) return fail("EXIT SENSOR BLOCKED",in.nowSec);
            retries=0;rpmSince=Double.NaN;setpointSince=in.nowSec;activeTarget=in.targetId;transition(State.PREPARE,in.nowSec);
        }
        if (state==State.IDLE) return Output.stopped("IDLE");
        double elapsed=in.nowSec-entered;
        if (state==State.WAIT_FOR_HIVE) {
            if(burstArmed) {
                if(!in.start||inventory.first()==null) {cancel();return Output.stopped("BURST COMPLETE / TRIGGER RELEASED");}
                ShotSelector.Selection next=select(in);
                if(next!=null)applySelection(next,in.nowSec);
                updateRpm(in);
                if(next!=null&&in.nowSec-lastExit>=burst.recoverySec&&!in.exit
                        &&in.hiveStable&&in.targetFresh&&observedFresh(in.nowSec,in.hiveObservedSec)
                        &&observedFresh(in.nowSec,in.targetObservedSec)&&in.hiveObservedSec>Math.max(entered,reacquireAfter)
                        &&in.targetObservedSec>Math.max(entered,reacquireAfter)&&flightSafe(in)) {
                    retries=0;activeTarget=in.targetId;transition(State.PREPARE,in.nowSec);
                }else {
                    // Pending flight time is an intentional wait; timeout starts after expected impact/settling.
                    double deadline=entered;
                    for(BurstPolicy.Flight f:flights)deadline=Math.max(deadline,f.arrivalMaxSec+flightSettling());
                    deadline=Math.max(deadline,reacquireAfter);
                    if(in.nowSec-deadline>timing.hive)return fail("BURST HIVE REACQUISITION TIMEOUT",in.nowSec);
                    return holding("BURST: RECOVER RPM / WAIT FOR SAFE HIVE IMPACT");
                }
            }else {
                // Never reuse the pre-release HIVE/aim readiness.
                if (in.hiveStable && in.targetFresh && observedFresh(in.nowSec,in.hiveObservedSec) && observedFresh(in.nowSec,in.targetObservedSec)
                        && in.hiveObservedSec>entered && in.targetObservedSec>entered
                        && elapsed>=recipe.settleSec) {
                    cancel();return Output.stopped("SHOT CONFIRMED; NEW START REQUIRED");
                }
                if (elapsed>timing.hive) return fail("HIVE REACQUISITION TIMEOUT",in.nowSec);
                return Output.stopped("WAIT FOR HIVE / REACQUIRE");
            }
        }
        if (state==State.REVERSE) {
            if (elapsed>=timing.reverse) { rpmSince=Double.NaN;transition(State.PREPARE,in.nowSec); }
            else return new Output(0,Double.NaN,Double.NaN,-reversePower,-reversePower,"BOUNDED JAM RECOVERY");
        }
        elapsed=in.nowSec-entered;
        boolean calibrated=true;
        if(state==State.PREPARE) {
            ShotSelector.Selection next=select(in);calibrated=next!=null;
            if(next!=null)applySelection(next,in.nowSec);
        }
        boolean range=!recipe.rangeCalibrated||(Double.isFinite(in.rangeInches)&&in.rangeInches>=recipe.minRange&&in.rangeInches<=recipe.maxRange);
        boolean aim=in.aimReady&&in.targetFresh&&(in.stationary||in.motionReady)&&in.hiveStable&&range&&calibrated
                &&(state==State.PREPARE||in.targetId==activeTarget)&&flightSafe(in)
                &&observedFresh(in.nowSec,in.hiveObservedSec)&&observedFresh(in.nowSec,in.targetObservedSec);
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT) {
            // Hold actuator settings during one release. A materially different solution cannot reuse them.
            ShotSelector.Selection current=select(in);
            aim &= current!=null&&sameSetpoints(recipe,current.recipe);
        }
        boolean rpmOk=updateRpm(in);
        if (state==State.PREPARE) {
            if (in.nowSec-entered>timing.prepare) return retryOrFault(in,"STAGING / READY TIMEOUT");
            boolean ready=aim&&in.staged&&!in.exit&&in.nowSec-setpointSince>=recipe.settleSec
                    && Double.isFinite(rpmSince)&&in.acquiredSec-rpmSince>=recipe.rpmStableSec;
            if (ready) {
                committedReleaseSec=Double.isFinite(in.releaseSec)?in.releaseSec:in.nowSec+releaseDelay(in);
                committedReleaseUncertainty=releaseUncertainty(in);
                releasePredictionKnown=Double.isFinite(in.releaseSec)||burst!=null;
                activeTarget=in.targetId;
                transition(State.FEED_ONE,in.nowSec);
            }
            else return new Output(recipe.rpm,recipe.hood,recipe.compression,in.staged?0:forwardPower,0,
                    !in.staged?"STAGE PIECE":!aim?"AIM / HIVE / MOTION INHIBIT":"SPIN / SERVO SETTLE");
        }
        if (!aim) {
            // A partly released piece is ambiguous. Never silently resume/retry it.
            return fail("READINESS LOST DURING RELEASE",in.nowSec);
        }
        if(!rpmOk&&state==State.FEED_ONE) {
            // A ball can load the wheel before the debounced exit edge arrives. Stop feed and await evidence.
            transition(State.CONFIRM_EXIT,in.nowSec);
            return holding("RPM DROOP / AWAIT CONFIRMED EXIT");
        }
        if (state==State.FEED_ONE) {
            if (in.nowSec-entered>=timing.feed) transition(State.CONFIRM_EXIT,in.nowSec);
            else return new Output(recipe.rpm,recipe.hood,recipe.compression,forwardPower,forwardPower,"FEED ONE");
        }
        if (state==State.CONFIRM_EXIT) {
            if (in.nowSec-entered>timing.exit) return fail("EXIT CONFIRMATION TIMEOUT; RECONCILE INVENTORY",in.nowSec);
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
    private ShotSelector.Selection select(Input in) {
        BallType type=inventory.first();
        ShotSelector.Selection s=in.selectedShot==null?selector.select(type,in.rangeInches):in.selectedShot;
        if(s!=null&&s.hasFlightMeasurement()&&!Double.isFinite(exitLatencySec))return null;
        return s==null||s.recipe.type!=type?null:s;
    }
    private void applySelection(ShotSelector.Selection next,double now) {
        if(recipe==null||!sameSetpoints(recipe,next.recipe)) {recipe=next.recipe;setpointSince=now;rpmSince=Double.NaN;}
        selection=new ShotSelector.Selection(recipe,next.flightSec,next.flightUncertaintySec);
    }
    private boolean sameSetpoints(ShotRecipe a,ShotRecipe b) {
        return a.type==b.type&&Math.abs(a.rpm-b.rpm)<=Math.min(a.rpmTolerance,b.rpmTolerance)*rpmRetargetFraction
                &&Math.abs(a.hood-b.hood)<=hoodRetargetTolerance&&Math.abs(a.compression-b.compression)<=compressionRetargetTolerance;
    }
    private Output holding(String reason) {return new Output(recipe.rpm,recipe.hood,recipe.compression,0,0,reason);}
    private void retireFlights(double now) {
        double settle=flightSettling();
        Iterator<BurstPolicy.Flight> it=flights.iterator();
        while(it.hasNext()) {
            BurstPolicy.Flight f=it.next();
            if(now>=f.arrivalMaxSec+settle) {
                reacquireAfter=Math.max(reacquireAfter,f.arrivalMaxSec+settle);it.remove();
            }
        }
    }
    private double flightSettling() {
        return Math.max(Double.isFinite(impactSettleSec)?impactSettleSec:0,burst==null?0:burst.impactSettleSec);
    }
    private boolean flightSafe(Input in) {
        if(burst==null)return flights.isEmpty()&&in.hiveObservedSec>reacquireAfter&&in.targetObservedSec>reacquireAfter;
        if(selection==null||!selection.hasFlightMeasurement()||flights.size()>=burst.maxInFlight)return false;
        for(BurstPolicy.Flight f:flights)if(!f.confirmed)return false;
        if(in.hiveObservedSec<=reacquireAfter||in.targetObservedSec<=reacquireAfter)return false;
        if(burst.impactModel==null)return flights.isEmpty();
        double uncertainty=selection.flightUncertaintySec+releaseUncertainty(in);
        double release=(state==State.FEED_ONE||state==State.CONFIRM_EXIT)&&Double.isFinite(committedReleaseSec)
                ?committedReleaseSec:Double.isFinite(in.releaseSec)?in.releaseSec:in.nowSec+releaseDelay(in);
        return burst.impactModel.safe(selection.recipe.type,in.targetId,in.hiveState,in.hiveObservedSec,
                release+selection.flightSec-uncertainty,release+selection.flightSec+uncertainty,inFlight());
    }
    private double releaseDelay(Input in) {return in.releaseDelaySec>0?in.releaseDelaySec:burst==null?0:burst.releaseDelaySec;}
    private void retainPossibleFlight() {
        if(selection!=null&&selection.hasFlightMeasurement()&&Double.isFinite(lastNow))
            flights.add(new BurstPolicy.Flight(recipe.type,activeTarget,lastNow,selection.flightSec,
                    selection.flightUncertaintySec+timing.feed+timing.exit,false));
    }
    private double releaseUncertainty(Input in) {return burst==null?in.releaseUncertaintySec:Math.max(in.releaseUncertaintySec,burst.releaseUncertaintySec);}
    private boolean updateRpm(Input in) {
        boolean ok=recipe!=null&&Double.isFinite(in.rpm)&&Math.abs(in.rpm-recipe.rpm)<=recipe.rpmTolerance;
        if(!ok)rpmSince=Double.NaN;
        else if(!Double.isFinite(rpmSince))rpmSince=in.acquiredSec;
        return ok;
    }
}
