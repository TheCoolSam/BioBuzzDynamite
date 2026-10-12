package org.firstinspires.ftc.teamcode.scoring;

import java.util.List;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Bounded burst settings; an optional measured model may authorize overlapping flights. */
public final class BurstPolicy {
    public static final class Flight {
        public final BallType type;
        public final int targetId;
        public final double exitedSec,arrivalMinSec,arrivalMaxSec;
        public final double releaseMinSec,releaseMaxSec;
        public final boolean confirmed;
        public Flight(BallType type,int targetId,double exit,double flight,double uncertainty) {
            this(type,targetId,exit,flight,uncertainty,true);
        }
        public Flight(BallType type,int targetId,double exit,double flight,double uncertainty,boolean confirmed) {
            this(type,targetId,exit,exit,exit,flight,uncertainty,confirmed);
        }
        public Flight(BallType type,int targetId,double exit,double releaseMin,double releaseMax,
                double flight,double uncertainty,boolean confirmed) {
            this.type=type;this.targetId=targetId;exitedSec=exit;
            releaseMinSec=releaseMin;releaseMaxSec=releaseMax;
            arrivalMinSec=releaseMin+flight-uncertainty;arrivalMaxSec=releaseMax+flight+uncertainty;
            this.confirmed=confirmed;
        }
    }
    public interface ImpactModel {
        /** Must include tip/target state changes from ALL pending pieces and arrival uncertainty.
         * A true result certifies the whole candidate arrival interval for this type and target.
         */
        /** Physical burst implementations must evaluate the complete shot and impact point. */
        default boolean safe(ShotTipContext context) {return false;}
        boolean safe(BallType next,int targetId,HiveTracker.State observed,double observedSec,
                double arrivalMinSec,double arrivalMaxSec,List<Flight> pending);
    }
    public final int maxInFlight;
    public final double recoverySec,impactSettleSec;
    public final double releaseDelaySec,releaseUncertaintySec;
    public final ImpactModel impactModel;
    public final boolean pollenBurstValidated,nectarBurstValidated;
    public boolean validated(BallType type) {return type==BallType.POLLEN?pollenBurstValidated:type==BallType.NECTAR&&nectarBurstValidated;}
    public BurstPolicy(int maxInFlight,double recoverySec,double impactSettleSec,
            double releaseDelaySec,double releaseUncertaintySec,ImpactModel impactModel) {
        this(maxInFlight,recoverySec,impactSettleSec,releaseDelaySec,releaseUncertaintySec,impactModel,false,false);
    }
    public BurstPolicy(int maxInFlight,double recoverySec,double impactSettleSec,double releaseDelaySec,
            double releaseUncertaintySec,ImpactModel impactModel,boolean pollenBurstValidated,boolean nectarBurstValidated) {
        this.pollenBurstValidated=pollenBurstValidated;this.nectarBurstValidated=nectarBurstValidated;
        if(maxInFlight<1||maxInFlight>4||!Double.isFinite(recoverySec)||recoverySec<=0
                ||!Double.isFinite(impactSettleSec)||impactSettleSec<=0
                ||!Double.isFinite(releaseDelaySec)||releaseDelaySec<=0
                ||!Double.isFinite(releaseUncertaintySec)||releaseUncertaintySec<=0||releaseUncertaintySec>=releaseDelaySec)
            throw new IllegalArgumentException("Measured bounded burst timing required");
        if(maxInFlight>1&&impactModel==null)throw new IllegalArgumentException("Overlapping flights require measured HIVE model");
        this.maxInFlight=maxInFlight;this.recoverySec=recoverySec;this.impactSettleSec=impactSettleSec;this.impactModel=impactModel;
        this.releaseDelaySec=releaseDelaySec;this.releaseUncertaintySec=releaseUncertaintySec;
    }
}
