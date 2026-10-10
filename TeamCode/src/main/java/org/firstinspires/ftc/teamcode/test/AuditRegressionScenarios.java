package org.firstinspires.ftc.teamcode.test;

import java.util.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.turret.*;

/** Behavior regressions for October 9 findings; no SDK/hardware dependency. */
public final class AuditRegressionScenarios {
    private static int checks;
    private static RobotState pose(double t) {
        return new RobotState(t, 60, 50, 0, 0, 0, 0, 0, 0, 0, true, true);
    }
    private static void check(boolean ok, String message) {
        checks++; if (!ok) throw new AssertionError(message);
    }
    public static void main(String[] args) {
        MapTipModel tips = new MapTipModel().set(1,0,1);
        TrackedPiece piece = new TrackedPiece(1,BallType.POLLEN,PieceOwnership.NEUTRAL,80,50,1,10,true);
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1}) {
            PickupPlanner p = new PickupPlanner(tips,(x,y,h,target)->invalid,new FixedShotSetupModel(.8));
            check(p.plan(pose(10),BallLoad.empty(),Arrays.asList(piece),10,null).getDecision()
                    == PickupPlan.Decision.WAIT,"invalid travel rejects action");
            p = new PickupPlanner(tips,new EuclideanTravelTimeModel(),(x,y,h,load)->invalid);
            check(p.plan(pose(10),new BallLoad(1,0),Collections.emptyList(),10,null).getDecision()
                    == PickupPlan.Decision.WAIT,"invalid shot rejects shoot-now");
        }
        PickupPlanner p = new PickupPlanner(tips,new EuclideanTravelTimeModel(),new FixedShotSetupModel(.8));
        check(p.plan(pose(0),BallLoad.empty(),Arrays.asList(piece),10,null).getDecision()
                == PickupPlan.Decision.INVALID,"stale pose rejected");
        check(p.plan(pose(11),BallLoad.empty(),Arrays.asList(piece),10,null).getDecision()
                == PickupPlan.Decision.INVALID,"future pose rejected");
        check(!TurretStateAdapter.toTurretState(pose(0),0,0,100,0,.02,10).poseValid,"turret freshness retained");
        check(!new RobotState(10,0,0,0,0,0,0,0,0,0,true,true,0,true,0).isFresh(10,.5),
                "new snapshot cannot refresh old acquisition");
        check(!new MapTipModel().hasEstimate(new BallLoad(1,0)),"unknown tip distinguishable");
        check(new MapTipModel().set(1,0,0).hasEstimate(new BallLoad(1,0)),"measured zero known");
        check(Double.isNaN(new PreferredPoseShotSetup().estimateSeconds(0,0,0,new BallLoad(1,0))),
                "missing scoring poses unavailable");
        RobotStateHistory history = new RobotStateHistory(); history.add(pose(0)); history.add(pose(.1));
        PieceTracker tracker = new PieceTracker(HomographyFloorProjection.fromCoefficients(
                new double[]{1,0,0,0,1,0,0,0,1}),PieceTrackerConstants.defaults());
        PieceObservation observation = PieceObservation.of(1,20,10,4,4,0);
        PieceTrackingResult first = tracker.update(Arrays.asList(observation),history,0);
        int id = first.getPieces().get(0).getId();
        for (int i=0;i<10;i++) {
            PieceTrackingResult same = tracker.update(Arrays.asList(observation),history,0);
            check(same.getPieces().get(0).getConfidence()==.2,"duplicate cannot mature");
        }
        tracker.update(Arrays.asList(PieceObservation.of(1,20,10,4,4,.1)),history,.1);
        PieceTrackingResult older = tracker.update(Arrays.asList(observation),history,.1);
        check(older.getPieces().get(0).getLastSeenTimestampSec()==.1,"older frame ignored");
        check(tracker.consume(id,.1),"confirmed consume removes track");
        check(tracker.update(Arrays.asList(observation),history,.1).getPieces().isEmpty(),"late frame cannot resurrect");
        check(tracker.update(Arrays.asList(PieceObservation.of(1,20,10,4,4,.2)),history,.2)
                .getPieces().isEmpty(),"consume tombstone suppresses pending evidence");
        tracker.reset(.2,1);
        check(tracker.size()==0,"reset clears tracks");
        check(tracker.update(Arrays.asList(observation),history,.2).getPieces().isEmpty(),"pre-reset evidence rejected");
        RobotStateHistory resetHistory = new RobotStateHistory(); resetHistory.add(pose(0));
        resetHistory.add(new RobotState(.1,80,50,0,0,0,0,0,0,0,true,true,.1,true,1));
        check(!resetHistory.sampleAt(.05).isPresent(),"history cannot interpolate across reset");
        resetHistory.add(pose(.2));
        check(resetHistory.sampleAt(.1).get().getResetGeneration()==1,"delayed old generation cannot erase new history");
        for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,-.1,1.1}) {
            boolean refused=false;try{new MapTipModel().set(1,0,invalid);}catch(IllegalArgumentException expected){refused=true;}
            check(refused,"invalid probability is not measured zero");
        }
        PieceTrackerConstants measured=PieceTrackerConstants.defaults().withMeasuredTiming(0,.15);
        check(measured.isTimingMeasured(),"zero latency may be explicitly measured");
        PieceTracker cadence=new PieceTracker(HomographyFloorProjection.fromCoefficients(new double[]{1,0,0,0,1,0,0,0,1}),measured);
        RobotStateHistory timing=new RobotStateHistory();timing.add(pose(0));timing.add(pose(.1));timing.add(pose(.2));
        cadence.update(Arrays.asList(observation),timing,0);
        check(cadence.update(Arrays.asList(PieceObservation.of(1,20,10,4,4,.1)),timing,.1).getPieces().get(0).getConfidence()==.2,
                "measured cadence prevents rapid evidence growth");
        check(cadence.update(Arrays.asList(PieceObservation.of(1,20,10,4,4,.2)),timing,.2).getPieces().get(0).getConfidence()==.4,
                "independent cadence permits evidence");
        System.out.println("AUDIT REGRESSION CHECKS PASSED ("+checks+")");
    }
}
