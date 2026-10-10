import java.util.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;

/** October 9 diagnostic probes. Prints observed behavior; does not control hardware. */
public final class FollowupAuditProbe {
    static RobotState robot(double t, double x, double y) {
        return new RobotState(t, x, y, 0, 0, 0, 0, 0, 0, 0, true, true);
    }
    public static void main(String[] args) {
        TrackedPiece piece = new TrackedPiece(1, BallType.POLLEN,
                PieceOwnership.NEUTRAL, 80, 50, 1, 10, true);
        MapTipModel tips = new MapTipModel().set(1, 0, 1);
        PickupPlanner planner = new PickupPlanner(tips,
                new EuclideanTravelTimeModel(), new FixedShotSetupModel(.8));
        PickupPlan stale = planner.plan(robot(0, 60, 50), BallLoad.empty(),
                Arrays.asList(piece), 10, null);
        System.out.println("10-second-old pose: " + stale.getDecision());

        PickupPlanner badTravel = new PickupPlanner(tips,
                (x, y, h, target) -> Double.NaN, new FixedShotSetupModel(.8));
        PickupPlan travel = badTravel.plan(robot(10, 60, 50), BallLoad.empty(),
                Arrays.asList(piece), 10, null);
        System.out.printf(Locale.US, "NaN travel: %s, seconds=%.1f, utility=%.9f%n",
                travel.getDecision(), travel.getEstimatedSeconds(), travel.getUtility());

        PickupPlanner badShot = new PickupPlanner(tips,
                new EuclideanTravelTimeModel(), (x, y, h, load) -> Double.NaN);
        PickupPlan shot = badShot.plan(robot(10, 60, 50), new BallLoad(1, 0),
                Collections.<TrackedPiece>emptyList(), 10, null);
        System.out.printf(Locale.US, "NaN shot setup: %s, seconds=%.1f, utility=%.9f%n",
                shot.getDecision(), shot.getEstimatedSeconds(), shot.getUtility());

        RobotStateHistory history = new RobotStateHistory();
        history.add(robot(0, 0, 0));
        history.add(robot(.1, 0, 0));
        history.add(robot(.2, 0, 0));
        PieceTracker tracker = new PieceTracker(HomographyFloorProjection.fromCoefficients(
                new double[]{1, 0, 0, 0, 1, 0, 0, 0, 1}), PieceTrackerConstants.defaults());
        PieceObservation observation = PieceObservation.of(1, 20, 10, 4, 4, .2);
        PieceTrackingResult first = tracker.update(Arrays.asList(observation), history, .2);
        PieceTrackingResult repeated = first;
        for (int i = 1; i < 5; i++) repeated = tracker.update(Arrays.asList(observation), history, .2);
        System.out.printf(Locale.US, "Five copies of one timestamp: confidence %.2f -> %.2f%n",
                first.getPieces().get(0).getConfidence(), repeated.getPieces().get(0).getConfidence());
        PieceTrackingResult older = tracker.update(Arrays.asList(
                PieceObservation.of(1, 20, 10, 4, 4, .1)), history, .2);
        System.out.printf(Locale.US, "Out-of-order observation: lastSeen %.2f -> %.2f%n",
                repeated.getPieces().get(0).getLastSeenTimestampSec(),
                older.getPieces().get(0).getLastSeenTimestampSec());
        PieceTrackingResult coast = tracker.update(Collections.<PieceObservation>emptyList(), history, .3);
        System.out.println("After collection/no detections: internal track still published="
                + !coast.getPieces().isEmpty());

        TrackedPiece edge = new TrackedPiece(2, BallType.POLLEN,
                PieceOwnership.NEUTRAL, 143, 50, 1, 10, true);
        PickupPlan wall = planner.plan(robot(10, 120, 50), BallLoad.empty(), Arrays.asList(edge), 10, null);
        System.out.printf(Locale.US, "Default wall model: %s, center=(%.1f, %.1f); right clearance=%.1f in%n",
                wall.getDecision(), wall.getEndpointX(), wall.getEndpointY(), 144-wall.getEndpointX());
    }
}
