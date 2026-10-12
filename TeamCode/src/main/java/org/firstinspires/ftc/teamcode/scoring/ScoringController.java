package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Nonblocking sensed shot cycle. Outputs are requests; final hardware gates still apply. */
public final class ScoringController {
    public enum State { IDLE, PREPARE, FEED_ONE, CONFIRM_EXIT, REVERSE, WAIT_FOR_HIVE, FAULT }
    public static final class Input {
        /** Positive autonomous action ID stays requested until accepted; zero uses the driver edge. */
        public long requestId;
        public double nowSec,acquiredSec,rpm;
        public double hiveObservedSec=Double.NaN,targetObservedSec=Double.NaN;
        public double rangeInches=Double.NaN;
        public ShotSelector.Selection selectedShot;
        public boolean motionReady;
        public boolean burst,preSpin;
        public LauncherFeedback launcher;
        public double targetHeight=Double.NaN;
        public org.firstinspires.ftc.teamcode.vision.pieces.TargetEstimate impact;
        public org.firstinspires.ftc.teamcode.planning.pickup.BallLoad carried;
        public int targetId;
        public HiveTracker.State hiveState=HiveTracker.State.UNKNOWN;
        public double releaseDelaySec,releaseUncertaintySec;
        public double releaseSec=Double.NaN;
        public boolean healthy,enabled,start,cancel,dismissWait,resetFault,staged,exit,aimReady,targetFresh,stationary,hiveStable;
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
    private double committedReleaseSec=Double.NaN,lastAcquired=Double.NaN,lastConfirmedReleaseSec=Double.NaN;
    private double exitLatencySec=Double.NaN,impactSettleSec=Double.NaN,committedReleaseUncertainty;
    private boolean releasePredictionKnown;
    private boolean burstArmed;
    private int burstConfirmed;
    private BallType burstType;
    private double rpmRetargetFraction=.1,hoodRetargetTolerance=.001,compressionRetargetTolerance=.001;
    private int activeTarget;
    private int retries;
    private long acceptedRequestId,completedRequestId,failedRequestId,activeRequestId;
    public long acceptedRequestId() {return acceptedRequestId;}
    public long completedRequestId() {return completedRequestId;}
    public long failedRequestId() {return failedRequestId;}
    private void failAction() {if(activeRequestId>0)failedRequestId=activeRequestId;activeRequestId=0;}
    private String fault="";
    private LauncherFeedback.Limits launcherLimits;
    private double pitchSince=Double.NaN,gapSince=Double.NaN,lastLauncherAcquired=Double.NaN;
    public boolean pitchReady,compressionReady,rpmReady;
    private double holdRpm=Double.NaN;
    /** Measured idle/hold velocity used between shots while a piece is staged or remaining. NaN keeps the stop-between-shots behavior. */
    public void setHoldRpm(double measuredHoldRpm) {
        if(state!=State.IDLE||!Double.isFinite(measuredHoldRpm)||measuredHoldRpm<=0)
            throw new IllegalArgumentException("Configure measured hold RPM before arming");
        holdRpm=measuredHoldRpm;
    }
    public void setLauncherLimits(LauncherFeedback.Limits limits) {
        if(state!=State.IDLE||limits==null)throw new IllegalArgumentException("Configure measured feedback limits before arming");
        launcherLimits=limits;
    }
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
    public double cadenceAt(double nextReleaseSec) {
        return Double.isFinite(lastConfirmedReleaseSec)&&nextReleaseSec>lastConfirmedReleaseSec
                ?nextReleaseSec-lastConfirmedReleaseSec:Double.NaN;
    }
    public ShotRecipe currentRecipe() {return recipe;}
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
        failAction();
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE) {
            retainPossibleFlight();
            inventory.markUncertain();fault="RELEASE CANCELLED; RECONCILE INVENTORY";state=State.FAULT;
        }else if(state!=State.FAULT)state=State.IDLE;
        recipe=null;selection=null;rpmSince=pitchSince=gapSince=Double.NaN;burstArmed=false;
        pitchReady=compressionReady=rpmReady=false;
    }
    private void resetFault() { fault="";state=State.IDLE;cancel(); }
    private void transition(State next,double now) { state=next;entered=now; }
    private Output fail(String message,double now) {
        failAction();
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT||state==State.REVERSE) {retainPossibleFlight();inventory.markUncertain();}
        pitchReady=compressionReady=rpmReady=false;fault=message;transition(State.FAULT,now);return Output.stopped(message);
    }
    public Output update(Input in) {
        boolean startEdge=in.start&&!previousStart,resetEdge=in.resetFault&&!previousReset;
        boolean actionPending=in.requestId>0&&in.requestId>acceptedRequestId;
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
            if(activeRequestId>0)completedRequestId=activeRequestId;
            activeRequestId=0;
            lastExit=in.nowSec;lastConfirmedReleaseSec=committedReleaseSec;
            if(burstArmed)burstConfirmed++;
            if(selection.hasFlightMeasurement())flights.add(new BurstPolicy.Flight(recipe.type,activeTarget,in.nowSec,
                    in.nowSec-exitLatencySec,in.nowSec,selection.flightSec,selection.flightUncertaintySec,true));
            transition(State.WAIT_FOR_HIVE,in.nowSec);rpmSince=Double.NaN;
            if(releasePredictionKnown&&(committedReleaseSec+committedReleaseUncertainty<in.nowSec-exitLatencySec
                    ||committedReleaseSec-committedReleaseUncertainty>in.nowSec))
                return fail("RELEASE TIMING OUTSIDE CALIBRATION",in.nowSec);
            if(burstArmed)return holding("SHOT CONFIRMED / BURST RECOVERY");
            if(in.preSpin)return holding("WAIT FOR HIVE / DELIBERATE PRE-SPIN");
            return idleOrHold(in,"WAIT FOR HIVE / REACQUIRE","WAIT FOR HIVE / HOLD");
        }
        if(exitEdge){inventory.markUncertain();return fail("UNEXPECTED EXIT; RECONCILE INVENTORY",in.nowSec);}
        if (state==State.IDLE && ((in.requestId==0&&startEdge)||actionPending)) {
            ShotSelector.Selection initial=select(in);
            if (initial==null) return Output.stopped("CONFIRMED PIECE / CALIBRATED RANGE REQUIRED");
            applySelection(initial,in.nowSec);
            if(in.burst&&recipe.physical()&&(burst==null||!burst.validated(recipe.type)))
                return Output.stopped("REQUESTED BURST REQUIRES SEPARATE PHYSICAL VALIDATION");
            burstArmed=burst!=null&&in.burst;burstConfirmed=0;burstType=recipe.type;
            if (in.exit) return fail("EXIT SENSOR BLOCKED",in.nowSec);
            activeRequestId=actionPending?in.requestId:0;
            if(activeRequestId>0)acceptedRequestId=activeRequestId;
            retries=0;activeTarget=in.targetId;transition(State.PREPARE,in.nowSec);
        }
        if (in.dismissWait && state==State.WAIT_FOR_HIVE) {
            transition(State.IDLE,in.nowSec);
        } else if (in.dismissWait && state==State.PREPARE) {
            cancel();
        }
        if (state==State.IDLE) {
            if(in.preSpin) {
                ShotSelector.Selection next=select(in);
                if(next!=null) {applySelection(next,in.nowSec);rpmReady=updateRpm(in)&&in.acquiredSec-rpmSince>=recipe.rpmStableSec;
                    updateLauncher(in);return holding("DELIBERATE PRE-SPIN / FEED DISABLED");}
            }
            return idleOrHold(in,"IDLE","IDLE HOLD");
        }
        double elapsed=in.nowSec-entered;
        if (state==State.WAIT_FOR_HIVE) {
            if(burstArmed) {
                if(!in.start||inventory.first()==null||(recipe.physical()&&(inventory.first()!=burstType
                        ||burstConfirmed>=(burstType==BallType.NECTAR?3:4)))) {cancel();return Output.stopped("BURST COMPLETE / TRIGGER RELEASED");}
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
                    cancel();return idleOrHold(in,"SHOT CONFIRMED; NEW START REQUIRED","SHOT CONFIRMED; HOLD");
                }
                if (elapsed>timing.hive) {
                    transition(State.IDLE,in.nowSec);
                    return idleOrHold(in,"SHOT CONFIRMED; HIVE WAIT RELEASED","SHOT CONFIRMED; HIVE WAIT RELEASED / HOLD");
                }
                if (in.preSpin) return holding("WAIT FOR HIVE / DELIBERATE PRE-SPIN");
                return idleOrHold(in,"WAIT FOR HIVE / REACQUIRE","WAIT FOR HIVE / HOLD");
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
        rpmReady=rpmOk&&Double.isFinite(rpmSince)&&in.acquiredSec-rpmSince>=recipe.rpmStableSec;
        boolean launcherReady=updateLauncher(in);
        if(state==State.FEED_ONE||state==State.CONFIRM_EXIT)aim &= launcherReady;
        if (state==State.PREPARE) {
            if (in.nowSec-entered>timing.prepare) return retryOrFault(in,"STAGING / READY TIMEOUT");
            boolean ready=aim&&launcherReady&&in.staged&&!in.exit&&in.nowSec-setpointSince>=recipe.settleSec
                    && Double.isFinite(rpmSince)&&in.acquiredSec-rpmSince>=recipe.rpmStableSec;
            if (ready) {
                committedReleaseSec=Double.isFinite(in.releaseSec)?in.releaseSec:in.nowSec+releaseDelay(in);
                committedReleaseUncertainty=releaseUncertainty(in);
                releasePredictionKnown=Double.isFinite(in.releaseSec)||burst!=null||recipe.physical();
                activeTarget=in.targetId;
                transition(State.FEED_ONE,in.nowSec);
            }
            else return new Output(recipe.rpm,recipe.hood,recipe.compression,in.staged?0:forwardPower,0,
                    !in.staged?"STAGE PIECE":!aim?"AIM / HIVE / MOTION INHIBIT":!pitchReady?"MEASURED PITCH ERROR / RATE":!compressionReady?"COMPRESSION ERROR / RATE":"SPIN / MEASURED SETTLE");
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
        if(s!=null&&s.recipe.physical()&&(!s.acceptsHeight(in.targetHeight)||in.impact==null||!in.impact.internalPoint||!s.hasReleaseMeasurement()||launcherLimits==null))return null;
        return s==null||s.recipe.type!=type?null:s;
    }
    private void applySelection(ShotSelector.Selection next,double now) {
        if(recipe==null||!sameSetpoints(recipe,next.recipe)) {recipe=next.recipe;setpointSince=now;rpmSince=pitchSince=gapSince=Double.NaN;}
        selection=new ShotSelector.Selection(recipe,next.flightSec,next.flightUncertaintySec,next.releaseDelaySec,
                next.releaseUncertaintySec,next.targetHeight,next.heightTolerance);
    }
    private boolean sameSetpoints(ShotRecipe a,ShotRecipe b) {
        return a.type==b.type&&a.geometry==b.geometry
                &&(!a.physical()||(Math.abs(a.pitchRad-b.pitchRad)<=launcherLimits.pitchError*.25
                    &&Math.abs(a.gapMm-b.gapMm)<=launcherLimits.gapError*.25))&&Math.abs(a.rpm-b.rpm)<=Math.min(a.rpmTolerance,b.rpmTolerance)*rpmRetargetFraction
                &&Math.abs(a.hood-b.hood)<=hoodRetargetTolerance&&Math.abs(a.compression-b.compression)<=compressionRetargetTolerance;
    }
    private Output holding(String reason) {return new Output(recipe.rpm,recipe.hood,recipe.compression,0,0,reason);}
    private boolean holdWanted(Input in) {
        return Double.isFinite(holdRpm)&&holdRpm>0&&in.enabled&&!in.cancel&&(in.staged||inventory.first()!=null);
    }
    private Output idleOrHold(Input in,String stoppedReason,String holdReason) {
        if(!holdWanted(in)) {
            if(state==State.IDLE) {
                recipe=null;selection=null;rpmSince=pitchSince=gapSince=Double.NaN;
                pitchReady=compressionReady=rpmReady=false;
            }
            return Output.stopped(stoppedReason);
        }
        ShotSelector.Selection next=select(in);
        if(next!=null)applySelection(next,in.nowSec);
        double hood=recipe==null?Double.NaN:recipe.hood,compression=recipe==null?Double.NaN:recipe.compression;
        if(recipe!=null) {updateRpm(in);updateLauncher(in);}
        return new Output(holdRpm,hood,compression,0,0,holdReason);
    }
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
        if(!burstArmed)return flights.isEmpty()&&in.hiveObservedSec>reacquireAfter&&in.targetObservedSec>reacquireAfter;
        if(selection==null||!selection.hasFlightMeasurement()||flights.size()>=burst.maxInFlight)return false;
        for(BurstPolicy.Flight f:flights)if(!f.confirmed)return false;
        if(in.hiveObservedSec<=reacquireAfter||in.targetObservedSec<=reacquireAfter)return false;
        if(burst.impactModel==null)return flights.isEmpty();
        double uncertainty=selection.flightUncertaintySec+releaseUncertainty(in);
        double release=(state==State.FEED_ONE||state==State.CONFIRM_EXIT)&&Double.isFinite(committedReleaseSec)
                ?committedReleaseSec:Double.isFinite(in.releaseSec)?in.releaseSec:in.nowSec+releaseDelay(in);
        if(selection.recipe.physical()) {
            if(in.carried==null||in.impact==null||!in.impact.internalPoint)return false;
            double cadence=cadenceAt(release);if(!Double.isFinite(cadence))cadence=burst.recoverySec;
            return burst.impactModel.safe(new ShotTipContext(in.carried,selection,in.impact,in.hiveState,cadence,
                    release+selection.flightSec-uncertainty,release+selection.flightSec+uncertainty,inFlight()));
        }
        return burst.impactModel.safe(selection.recipe.type,in.targetId,in.hiveState,in.hiveObservedSec,
                release+selection.flightSec-uncertainty,release+selection.flightSec+uncertainty,inFlight());
    }
    private double releaseDelay(Input in) {if(selection!=null&&selection.hasReleaseMeasurement())return selection.releaseDelaySec;return in.releaseDelaySec>0?in.releaseDelaySec:burst==null?0:burst.releaseDelaySec;}
    private void retainPossibleFlight() {
        if(selection!=null&&selection.hasFlightMeasurement()&&Double.isFinite(lastNow))
            flights.add(new BurstPolicy.Flight(recipe.type,activeTarget,lastNow,selection.flightSec,
                    selection.flightUncertaintySec+timing.feed+timing.exit,false));
    }
    private double releaseUncertainty(Input in) {if(selection!=null&&selection.hasReleaseMeasurement())return selection.releaseUncertaintySec;return burst==null?in.releaseUncertaintySec:Math.max(in.releaseUncertaintySec,burst.releaseUncertaintySec);}
    private boolean updateLauncher(Input in) {
        // Legacy normalized recipes exist only for desktop simulation. Hardware rejects them.
        if(!recipe.physical()) {pitchReady=compressionReady=true;return true;}
        LauncherFeedback s=in.launcher;
        boolean monotone=s!=null&&(!Double.isFinite(lastLauncherAcquired)||s.acquiredSec>=lastLauncherAcquired);
        pitchReady=monotone&&launcherLimits!=null&&launcherLimits.pitchReady(s,recipe,in.nowSec);
        compressionReady=monotone&&launcherLimits!=null&&launcherLimits.compressionReady(s,recipe,in.nowSec);
        if(!pitchReady)pitchSince=Double.NaN;
        else if(!Double.isFinite(pitchSince))pitchSince=s.acquiredSec;
        if(!compressionReady)gapSince=Double.NaN;
        else if(!Double.isFinite(gapSince))gapSince=s.acquiredSec;
        if(s!=null&&Double.isFinite(s.acquiredSec)&&monotone)lastLauncherAcquired=s.acquiredSec;
        return pitchReady&&compressionReady&&s.acquiredSec-pitchSince>=launcherLimits.stableSec
                &&s.acquiredSec-gapSince>=launcherLimits.stableSec;
    }
    private boolean updateRpm(Input in) {
        boolean ok=recipe!=null&&Double.isFinite(in.rpm)&&Math.abs(in.rpm-recipe.rpm)<=recipe.rpmTolerance;
        if(!ok)rpmSince=Double.NaN;
        else if(!Double.isFinite(rpmSince))rpmSince=in.acquiredSec;
        return ok;
    }
}
