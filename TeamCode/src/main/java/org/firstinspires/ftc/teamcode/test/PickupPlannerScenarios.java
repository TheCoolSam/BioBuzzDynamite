package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.planning.pickup.BallLoad;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.CaptureGeometry;
import org.firstinspires.ftc.teamcode.planning.pickup.EuclideanTravelTimeModel;
import org.firstinspires.ftc.teamcode.planning.pickup.FixedShotSetupModel;
import org.firstinspires.ftc.teamcode.planning.pickup.MapTipModel;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlan;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlanner;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlannerConstants;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupTarget;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
import org.firstinspires.ftc.teamcode.planning.pickup.PreferredPoseShotSetup;
import org.firstinspires.ftc.teamcode.planning.pickup.ShotSetupTimeModel;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Desktop scenarios for the pickup planner. Not an OpMode.
 * Tip numbers in this file are fixtures for the tests, not robot assumptions.
 */
public final class PickupPlannerScenarios {

    private static int checks = 0;

    private PickupPlannerScenarios() {
    }

    public static void main(String[] args) {
        testNearestIsStrategicallyWrong();
        testClosestPiecesWinOnTime();
        testMixedLoadOnTheWay();
        testOrderIsNotListOrder();
        testAlreadyReadyToShoot();
        testCapacity();
        testIllegalAndStalePieces();
        testConfidenceCanRejectAShortRoute();
        testHysteresis();
        testCommittedGeometryRefreshes();
        testCommittedCostsRefresh();
        testCompletedShotInvalidatesCommitment();
        testInventoryChangeInvalidatesCommitment();
        testFutureObservationsAndInvalidClock();
        testCommitmentOutsideAdmissionPool();
        testTargetDisappears();
        testInvalidPoseDoesNotUseOrigin();
        testCollectThenReplan();
        testReservedSlotKeepsTheOtherType();
        testRouteFinishingNearTheShotWins();
        testShootNowUsesShotTravel();
        testNectarSurvivesAPollenCrowd();
        testPollenSurvivesANectarCrowd();
        testTravelTimeUsesTheLongerAxis();
        System.out.println("PICKUP PLANNER CHECKS PASSED (" + checks + ")");
    }

    /** Two close pollen are almost useless. Three farther nectar are not. */
    private static void testNearestIsStrategicallyWrong() {
        MapTipModel tips = new MapTipModel()
                .set(2, 0, 0.05)
                .set(0, 3, 0.95);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> pieces = Arrays.asList(
                ball(1, BallType.POLLEN, 12.0, 0.0),
                ball(2, BallType.POLLEN, 18.0, 0.0),
                ball(3, BallType.NECTAR, 36.0, 0.0),
                ball(4, BallType.NECTAR, 40.0, 0.0),
                ball(5, BallType.NECTAR, 45.0, 0.0));

        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(plan.getDecision() == PickupPlan.Decision.PICKUP, "A picks up");
        check(plan.getTargets().size() == 3, "A takes three pieces");
        check(allType(plan, BallType.NECTAR), "A chooses nectar despite the longer drive");
        check(plan.getTipProbability() > 0.9, "A tip is the nectar load");

        PickupTarget first = plan.getTargets().get(0);
        double heading = first.getApproachHeadingRad();
        near(first.getCaptureX(), first.getPiece().getFieldX() + (Math.cos(heading) * PickupPlannerConstants.CAPTURE_LEAD_INCHES),
                1e-9, "A capture is past the piece");
        near(first.getCaptureY(), first.getPiece().getFieldY() + (Math.sin(heading) * PickupPlannerConstants.CAPTURE_LEAD_INCHES),
                1e-9, "A capture y is past the piece");
    }

    /** Both loads tip well. The short pollen route wins on time. */
    private static void testClosestPiecesWinOnTime() {
        MapTipModel tips = new MapTipModel()
                .set(1, 0, 0.01)
                .set(2, 0, 0.05)
                .set(3, 0, 0.15)
                .set(4, 0, 0.90)
                .set(0, 1, 0.02)
                .set(0, 2, 0.10)
                .set(0, 3, 0.95);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> pieces = Arrays.asList(
                ball(1, BallType.POLLEN, 12.0, 0.0),
                ball(2, BallType.POLLEN, 24.0, 0.0),
                ball(3, BallType.POLLEN, 36.0, 0.0),
                ball(4, BallType.POLLEN, 48.0, 0.0),
                ball(5, BallType.NECTAR, 150.0, 0.0),
                ball(6, BallType.NECTAR, 170.0, 0.0),
                ball(7, BallType.NECTAR, 190.0, 0.0));

        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(plan.getTargets().size() == 4, "B takes four pollen");
        check(allType(plan, BallType.POLLEN), "B leaves the distant nectar");
    }

    /** A pollen on the way is worth grabbing. The same pollen off the route is not. */
    private static void testMixedLoadOnTheWay() {
        MapTipModel tips = new MapTipModel()
                .set(0, 1, 0.02)
                .set(0, 2, 0.10)
                .set(0, 3, 0.80)
                .set(1, 0, 0.01)
                .set(1, 1, 0.05)
                .set(1, 2, 0.15)
                .set(1, 3, 0.90);
        PickupPlanner planner = planner(tips);
        TrackedPiece nectarA = ball(1, BallType.NECTAR, 48.0, 0.0);
        TrackedPiece nectarB = ball(2, BallType.NECTAR, 64.0, 0.0);
        TrackedPiece nectarC = ball(3, BallType.NECTAR, 80.0, 0.0);
        TrackedPiece onTheWay = ball(4, BallType.POLLEN, 36.0, 0.0);
        TrackedPiece farOff = ball(4, BallType.POLLEN, 36.0, 96.0);

        PickupPlan mixed = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(nectarA, nectarB, nectarC, onTheWay),
                0.0,
                null);
        check(mixed.getTargets().size() == 4, "C grabs the pollen that is already on the route");
        check(countType(mixed, BallType.NECTAR) == 3, "C still takes every nectar");
        check(countType(mixed, BallType.POLLEN) == 1, "C adds the one pollen");

        PickupPlan nectarOnly = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(nectarA, nectarB, nectarC, farOff),
                0.0,
                null);
        check(nectarOnly.getTargets().size() == 3, "C leaves a far pollen");
        check(allType(nectarOnly, BallType.NECTAR), "C far case is nectar only");
    }

    /** Three pieces. The fastest permutation must beat the order they were listed in. */
    private static void testOrderIsNotListOrder() {
        MapTipModel tips = new MapTipModel().set(0, 3, 0.90).set(0, 2, 0.05).set(0, 1, 0.01);
        PickupPlanner planner = planner(tips);
        RobotState robot = pose(0.0, 0.0);
        TrackedPiece north = ball(10, BallType.NECTAR, 0.0, 36.0);
        TrackedPiece east = ball(20, BallType.NECTAR, 36.0, 0.0);
        TrackedPiece corner = ball(30, BallType.NECTAR, 36.0, 36.0);
        List<TrackedPiece> listed = Arrays.asList(north, east, corner);

        PickupPlan plan = planner.plan(robot, BallLoad.empty(), listed, 0.0, null);
        check(plan.getTargets().size() == 3, "D takes all three");

        int[] planned = ids(plan);
        int[] fastest = fastestOrder(robot, listed);
        check(Arrays.equals(planned, fastest), "D matches the lowest-time order " + Arrays.toString(fastest)
                + " but planned " + Arrays.toString(planned));

        int[] listedOrder = new int[] {10, 20, 30};
        double listedTime = routeSeconds(robot, piecesInOrder(listed, listedOrder));
        double plannedTravel = routeSeconds(robot, piecesInOrder(listed, planned));
        check(!Arrays.equals(planned, listedOrder), "D does not keep the input order");
        check(plannedTravel < listedTime - 1.0e-6, "D the chosen order is faster than the listed order");
    }

    /** A load that already tips well is not sent across the field for a tiny gain. */
    private static void testAlreadyReadyToShoot() {
        MapTipModel tips = new MapTipModel()
                .set(0, 3, 0.97)
                .set(1, 3, 0.98);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> farPollen = Arrays.asList(ball(1, BallType.POLLEN, 120.0, 0.0));
        PickupPlan plan = planner.plan(
                pose(0.0, 0.0),
                new BallLoad(0, 3),
                farPollen,
                0.0,
                null);
        check(plan.getDecision() == PickupPlan.Decision.SHOOT_NOW, "E shoots now");
        check(plan.getTargets().isEmpty(), "E drives nowhere");
        check(plan.getResultingLoad().getNectar() == 3, "E keeps the onboard nectar");
        check(plan.getResultingLoad().getPollen() == 0, "E does not pretend it collected");
    }

    /** Two pieces already aboard. At most two more targets. */
    private static void testCapacity() {
        MapTipModel tips = new MapTipModel()
                .set(2, 0, 0.10)
                .set(2, 1, 0.40)
                .set(2, 2, 0.95)
                .set(2, 3, 0.99);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> nectar = new ArrayList<TrackedPiece>();
        for (int i = 0; i < 5; i++) {
            nectar.add(ball(i + 1, BallType.NECTAR, 20.0 + (18.0 * i), 0.0));
        }
        PickupPlan plan = planner.plan(pose(0.0, 0.0), new BallLoad(2, 0), nectar, 0.0, null);
        check(plan.getTargets().size() <= 2, "F never exceeds remaining capacity");
        check(plan.getTargets().size() == 2, "F uses the two open slots");
        check(plan.getResultingLoad().total() == 4, "F ends at capacity");
        check(plan.getResultingLoad().total() <= PickupPlannerConstants.MAX_CAPACITY, "F load fits");
    }

    /** Illegal, opponent-but-allowed, and stale pieces. */
    private static void testIllegalAndStalePieces() {
        MapTipModel tips = new MapTipModel()
                .set(0, 1, 0.90)
                .set(1, 0, 0.20);
        PickupPlanner planner = planner(tips);
        TrackedPiece illegal = new TrackedPiece(
                1, BallType.NECTAR, PieceOwnership.OPPONENT, 12.0, 0.0, 1.0, 0.0, false);
        TrackedPiece legalPollen = ball(2, BallType.POLLEN, 30.0, 0.0);
        PickupPlan skipped = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(illegal, legalPollen),
                0.0,
                null);
        check(!containsId(skipped, 1), "G never selects the illegal piece");
        check(containsId(skipped, 2), "G still takes the legal piece");

        TrackedPiece opponentAllowed = new TrackedPiece(
                7, BallType.NECTAR, PieceOwnership.OPPONENT, 16.0, 0.0, 1.0, 0.0, true);
        PickupPlan allowed = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(opponentAllowed, legalPollen),
                0.0,
                null);
        check(containsId(allowed, 7), "G can collect an opponent piece when the caller marked it legal");

        TrackedPiece stale = new TrackedPiece(
                9, BallType.NECTAR, PieceOwnership.ALLIANCE, 12.0, 0.0, 1.0, 0.0, true);
        PickupPlan ignored = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(stale),
                5.0,
                null);
        check(ignored.getDecision() == PickupPlan.Decision.SHOOT_NOW, "G drops a stale detection");
        check(!containsId(ignored, 9), "G stale id is absent");
    }

    /**
     * A slightly longer route of reliable detections beats a short route of
     * very poor ones. Raise the poor confidences and the short route wins again.
     */
    private static void testConfidenceCanRejectAShortRoute() {
        MapTipModel tips = new MapTipModel().set(2, 0, 0.90).set(0, 2, 0.90);
        PickupPlanner planner = planner(tips);
        TrackedPiece poorA = seen(1, BallType.POLLEN, 16.0, 0.0, 0.05);
        TrackedPiece poorB = seen(2, BallType.POLLEN, 30.0, 0.0, 0.05);
        TrackedPiece goodA = seen(3, BallType.NECTAR, 24.0, 0.0, 0.98);
        TrackedPiece goodB = seen(4, BallType.NECTAR, 42.0, 0.0, 0.98);

        PickupPlan pollenOnly = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(poorA, poorB),
                0.0,
                null);
        PickupPlan both = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(poorA, poorB, goodA, goodB),
                0.0,
                null);
        check(allType(both, BallType.NECTAR), "H prefers the reliable nectar");
        check(both.getEstimatedSeconds() > pollenOnly.getEstimatedSeconds(),
                "H winner is actually the longer route");

        TrackedPiece sureA = seen(1, BallType.POLLEN, 16.0, 0.0, 0.98);
        TrackedPiece sureB = seen(2, BallType.POLLEN, 30.0, 0.0, 0.98);
        PickupPlan reliableNear = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(sureA, sureB, goodA, goodB),
                0.0,
                null);
        check(allType(reliableNear, BallType.POLLEN), "H near route wins once its confidence is real");
    }

    /** A slightly shorter alternative should not cause chatter; a much shorter one should. */
    private static void testHysteresis() {
        MapTipModel tips = new MapTipModel().set(1, 0, 0.80);
        PickupPlanner planner = planner(tips);
        TrackedPiece first = ball(1, BallType.POLLEN, 30.0, 0.0);
        TrackedPiece slightlyCloser = ball(2, BallType.POLLEN, 28.0, 0.0);
        PickupPlan fresh = planner.plan(pose(0.0, 0.0), BallLoad.empty(), Arrays.asList(first), 0.0, null);
        check(fresh.getUtility() > 0.0, "I has a positive utility");
        List<TrackedPiece> closeChoices = Arrays.asList(first, slightlyCloser);
        PickupPlan unconstrained = planner.plan(pose(0.0, 0.0), BallLoad.empty(), closeChoices, 0.0, null);
        check(containsId(unconstrained, 2), "I alternate is better without a commitment");
        check(unconstrained.getUtility() < fresh.getUtility() * 1.15, "I improvement is below threshold");
        PickupPlan kept = planner.plan(pose(0.0, 0.0), BallLoad.empty(), closeChoices, 0.0, fresh);
        check(containsId(kept, 1), "I keeps the target for a small improvement");

        TrackedPiece muchCloser = ball(2, BallType.POLLEN, 10.0, 0.0);
        List<TrackedPiece> betterChoices = Arrays.asList(first, muchCloser);
        PickupPlan switched = planner.plan(pose(0.0, 0.0), BallLoad.empty(), betterChoices, 0.0, fresh);
        check(containsId(switched, 2), "I switches for a large improvement");
        check(switched.getUtility() > fresh.getUtility() * 1.15, "I switch beats threshold");
        PickupPlan alteredUtility = planner.plan(pose(0.0, 0.0), BallLoad.empty(),
                betterChoices, 0.0, fresh.withUtility(1.0e6));
        check(containsId(alteredUtility, 2), "I a stale stored utility cannot prevent a switch");
    }

    private static void testCommittedGeometryRefreshes() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8));
        PickupPlan original = planner.plan(pose(0, 0), BallLoad.empty(),
                Arrays.asList(ball(1, BallType.POLLEN, 20, 0)), 0, null);
        TrackedPiece moved = new TrackedPiece(1, BallType.POLLEN, PieceOwnership.NEUTRAL,
                25, 0, 0.7, 0.1, true);
        PickupPlan refreshed = planner.plan(pose(0, 0), BallLoad.empty(), Arrays.asList(moved), 0.1, original);
        near(refreshed.getTargets().get(0).getPiece().getFieldX(), 25, 1e-9, "moving target uses latest point");
        near(refreshed.getEndpointX(), 31, 1e-9, "capture endpoint follows moving target");
        check(refreshed.getEstimatedSeconds() > original.getEstimatedSeconds(), "moving target updates remaining time");
        check(refreshed.getPlanConfidence() < original.getPlanConfidence(), "fresh confidence updates utility");
        near(original.getEndpointX(), 26, 1e-9, "refresh preserves original immutable plan");

        PickupPlan advanced = planner.plan(pose(10, 0), BallLoad.empty(), Arrays.asList(moved), 0.1, refreshed);
        PickupPlan baseline = planner.plan(pose(10, 0), BallLoad.empty(), Arrays.asList(moved), 0.1, null);
        near(advanced.getEstimatedSeconds(), baseline.getEstimatedSeconds(), 1e-9, "progress uses current robot pose");
    }

    private static void testCommittedCostsRefresh() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8));
        PickupPlan original = planner.plan(pose(0, 0), BallLoad.empty(),
                Arrays.asList(ball(1, BallType.POLLEN, 10, 0)), 0, null);
        PickupPlan next = planner.plan(pose(0, 0), BallLoad.empty(), Arrays.asList(
                ball(1, BallType.POLLEN, 100, 0), ball(2, BallType.POLLEN, 30, 0)), 0, original);
        check(containsId(next, 2), "a worsened incumbent is compared at its current cost");
    }

    private static void testCompletedShotInvalidatesCommitment() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8));
        PickupPlan shoot = planner.plan(pose(0, 0), new BallLoad(1, 0),
                new ArrayList<TrackedPiece>(), 0, null);
        check(shoot.getDecision() == PickupPlan.Decision.SHOOT_NOW, "loaded robot shoots");
        PickupPlan next = planner.plan(pose(0, 0), BallLoad.empty(),
                Arrays.asList(ball(1, BallType.POLLEN, 20, 0)), 0, shoot);
        check(next.getDecision() == PickupPlan.Decision.PICKUP, "unloading invalidates completed shoot commitment");
        check(next.getResultingLoad().getPollen() == 1, "next load comes from new collection");
        PickupPlan relocated = planner.plan(pose(40, 0), new BallLoad(1, 0),
                new ArrayList<TrackedPiece>(), 0, shoot);
        near(relocated.getEndpointX(), 40, 1e-9, "shoot setup also refreshes its current pose");
    }

    private static void testInventoryChangeInvalidatesCommitment() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8).set(0, 1, 0.8));
        PickupPlan committed = planner.plan(pose(0, 0), new BallLoad(1, 0),
                new ArrayList<TrackedPiece>(), 0, null);
        PickupPlan changed = planner.plan(pose(0, 0), new BallLoad(0, 1),
                new ArrayList<TrackedPiece>(), 0, committed);
        check(changed.getResultingLoad().getPollen() == 0, "inventory composition change drops old pollen");
        check(changed.getResultingLoad().getNectar() == 1, "inventory composition change uses actual nectar");
        check(planner.plan(pose(0, 0), new BallLoad(5, 0), new ArrayList<TrackedPiece>(), 0, null)
                .getDecision() == PickupPlan.Decision.INVALID, "over-capacity load does not plan more actions");
    }

    private static void testFutureObservationsAndInvalidClock() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8));
        TrackedPiece future = new TrackedPiece(1, BallType.POLLEN, PieceOwnership.NEUTRAL,
                20, 0, 1, 100, true);
        PickupPlan plan = planner.plan(pose(0, 0), BallLoad.empty(), Arrays.asList(future), 0, null);
        check(plan.getTargets().isEmpty(), "future observation is rejected");
        check(planner.plan(pose(0, 0), new BallLoad(1, 0), Arrays.asList(future), Double.NaN, null)
                .getDecision() == PickupPlan.Decision.INVALID, "invalid clock cannot produce a shoot command");
        check(planner.plan(pose(0, 0), BallLoad.empty(), Arrays.asList(future), Double.POSITIVE_INFINITY, null)
                .getDecision() == PickupPlan.Decision.INVALID, "infinite clock is rejected");
        PickupPlan current = planner.plan(pose(0, 0), BallLoad.empty(),
                Arrays.asList(ball(2, BallType.POLLEN, 20, 0)), 0, null);
        check(current.getDecision() == PickupPlan.Decision.PICKUP, "observation at current time remains usable");
    }

    private static void testCommitmentOutsideAdmissionPool() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 0.8));
        TrackedPiece incumbent = ball(100, BallType.POLLEN, 30, 0);
        PickupPlan original = planner.plan(pose(0, 0), BallLoad.empty(), Arrays.asList(incumbent), 0, null);
        List<TrackedPiece> crowded = new ArrayList<TrackedPiece>();
        for (int i = 0; i < PickupPlannerConstants.MAX_CANDIDATES; i++) {
            crowded.add(ball(i + 1, BallType.POLLEN, 28, 0));
        }
        crowded.add(incumbent);
        PickupPlan next = planner.plan(pose(0, 0), BallLoad.empty(), crowded, 0, original);
        check(containsId(next, 100), "pool pruning does not invalidate a fresh committed target");
    }

    /** A missing first target replans even if the old utility looks enormous. */
    private static void testTargetDisappears() {
        MapTipModel tips = new MapTipModel().set(0, 1, 0.50).set(0, 2, 0.90);
        PickupPlanner planner = planner(tips);
        TrackedPiece first = ball(1, BallType.NECTAR, 24.0, 0.0);
        TrackedPiece second = ball(2, BallType.NECTAR, 42.0, 0.0);
        List<TrackedPiece> both = Arrays.asList(first, second);
        PickupPlan committed = planner.plan(pose(0.0, 0.0), BallLoad.empty(), both, 0.0, null)
                .withUtility(1.0e6);
        check(committed.getTargets().get(0).getPiece().getId() == 1, "J setup starts at piece 1");

        PickupPlan replanned = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(second),
                0.0,
                committed);
        check(replanned != committed, "J does not keep the dead route");
        check(!containsId(replanned, 1), "J drops the piece that disappeared");
        check(containsId(replanned, 2), "J retargets the piece that is still there");
    }

    /** An invalid pose must not be treated as the origin or as its own stale coordinates. */
    private static void testInvalidPoseDoesNotUseOrigin() {
        MapTipModel tips = new MapTipModel().set(0, 1, 0.90);
        PickupPlanner planner = planner(tips);
        TrackedPiece nearOrigin = ball(1, BallType.NECTAR, 12.0, 0.0);
        TrackedPiece nearRobot = ball(2, BallType.NECTAR, 100.0, 0.0);
        List<TrackedPiece> pieces = Arrays.asList(nearOrigin, nearRobot);

        PickupPlan fromOrigin = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        PickupPlan fromReal = planner.plan(pose(90.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(containsId(fromOrigin, 1), "K origin would take the near piece");
        check(containsId(fromReal, 2), "K a real pose at x=90 would take the far piece");

        RobotState blind = new RobotState(
                1.0, 90.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                false, true);
        PickupPlan held = planner.plan(blind, BallLoad.empty(), pieces, 1.0, null);
        check(held.getDecision() == PickupPlan.Decision.INVALID, "K invalid pose does not plan motion");
        check(held.getTargets().isEmpty(), "K invalid pose has no targets");
        check(!containsId(held, 1), "K did not snap to the origin");
        check(!containsId(held, 2), "K did not use the invalid coordinates");
    }

    /** After one grab, the caller updates the load and the piece list, then plans again. */
    private static void testCollectThenReplan() {
        MapTipModel tips = new MapTipModel().set(0, 1, 0.02).set(0, 2, 0.15).set(0, 3, 0.90);
        PickupPlanner planner = planner(tips);
        TrackedPiece a = ball(1, BallType.NECTAR, 20.0, 0.0);
        TrackedPiece b = ball(2, BallType.NECTAR, 40.0, 0.0);
        TrackedPiece c = ball(3, BallType.NECTAR, 60.0, 0.0);
        PickupPlan first = planner.plan(
                pose(0.0, 0.0),
                BallLoad.empty(),
                Arrays.asList(a, b, c),
                0.0,
                null);
        check(first.getTargets().size() == 3, "collect setup takes three");
        int grabbed = first.getTargets().get(0).getPiece().getId();

        List<TrackedPiece> remaining = new ArrayList<TrackedPiece>();
        remaining.add(a);
        remaining.add(b);
        remaining.add(c);
        for (int i = remaining.size() - 1; i >= 0; i--) {
            if (remaining.get(i).getId() == grabbed) {
                remaining.remove(i);
            }
        }
        PickupPlan next = planner.plan(
                pose(0.0, 0.0),
                new BallLoad(0, 1),
                remaining,
                0.0,
                first);
        check(!containsId(next, grabbed), "collected piece is not targeted again");
        check(next.getTargets().size() == 2, "the other two are still worth taking");
        check(next.getResultingLoad().getNectar() == 3, "onboard count plus the route is three");
    }

    /**
     * More pollen than the pool can hold, plus one far nectar. The nectar keeps
     * a reserved seat, so a tip that only that nectar can produce is still found.
     */
    private static void testReservedSlotKeepsTheOtherType() {
        MapTipModel tips = new MapTipModel().set(0, 1, 0.99);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> crowded = new ArrayList<TrackedPiece>();
        for (int i = 0; i < PickupPlannerConstants.MAX_CANDIDATES; i++) {
            crowded.add(ball(i + 1, BallType.POLLEN, 8.0 + (8.0 * i), 0.0));
        }
        crowded.add(ball(100, BallType.NECTAR, 220.0, 0.0));
        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), crowded, 0.0, null);
        check(containsId(plan, 100), "reserved slot keeps the lone nectar");
        check(plan.getResultingLoad().getNectar() == 1, "the nectar is the load that tips");
    }

    /**
     * Same load and tip chance. The route that finishes next to the injected
     * shot wins even though collecting it takes longer.
     */
    private static void testRouteFinishingNearTheShotWins() {
        MapTipModel tips = new MapTipModel().set(1, 0, 0.80);
        PreferredPoseShotSetup.ShotPose shot = new PreferredPoseShotSetup.ShotPose(0.0, 0.0, 0.0);
        PickupPlanner planner = planner(tips, new PreferredPoseShotSetup(shot));
        RobotState robot = pose(0.0, 60.0);
        TrackedPiece farFinish = ball(1, BallType.POLLEN, 40.0, 60.0);
        TrackedPiece nearFinish = ball(2, BallType.POLLEN, 0.0, 10.0);

        double collectFar = routeSeconds(robot, Arrays.asList(farFinish));
        double collectNear = routeSeconds(robot, Arrays.asList(nearFinish));
        check(collectNear > collectFar, "L the near-shot piece takes longer to collect");

        PickupPlan onlyFar = planner.plan(robot, BallLoad.empty(), Arrays.asList(farFinish), 0.0, null);
        PickupPlan onlyNear = planner.plan(robot, BallLoad.empty(), Arrays.asList(nearFinish), 0.0, null);
        check(onlyNear.getEstimatedSeconds() < onlyFar.getEstimatedSeconds(),
                "L finishing beside the shot lowers time-to-tip");
        check(distanceToShot(onlyNear, shot) < distanceToShot(onlyFar, shot),
                "L the winning endpoint is closer to the shot");

        PickupPlan both = planner.plan(
                robot,
                BallLoad.empty(),
                Arrays.asList(farFinish, nearFinish),
                0.0,
                null);
        check(containsId(both, 2), "L chooses the route that ends near the shot");
        check(!containsId(both, 1), "L leaves the fast pickup that ends far away");
        check(both.getTargets().size() == 1, "L takes the one pollen, not both");
    }

    /**
     * A loaded robot far from the shot pays the drive. Shoot-now is not a
     * 0.25 second cycle. A piece that ends beside the shot can beat that drive.
     */
    private static void testShootNowUsesShotTravel() {
        PreferredPoseShotSetup.ShotPose shot = new PreferredPoseShotSetup.ShotPose(120.0, 0.0, 0.0);
        ShotSetupTimeModel shots = new PreferredPoseShotSetup(shot);
        MapTipModel tips = new MapTipModel().set(0, 3, 0.90).set(0, 2, 0.30);
        PickupPlanner planner = planner(tips, shots);
        RobotState robot = pose(0.0, 0.0);

        PickupPlan shoot = planner.plan(robot, new BallLoad(0, 3), new ArrayList<TrackedPiece>(), 0.0, null);
        check(shoot.getDecision() == PickupPlan.Decision.SHOOT_NOW, "M shoots the onboard load");
        near(shoot.getEndpointX(), 0.0, 1e-9, "M shoot-now starts from the robot");
        near(shoot.getEndpointY(), 0.0, 1e-9, "M shoot-now y is the robot");
        double expected = shots.estimateSeconds(0.0, 0.0, 0.0, new BallLoad(0, 3));
        near(shoot.getEstimatedSeconds(), expected, 1e-9, "M time is the shot model");
        check(shoot.getEstimatedSeconds() > 2.0, "M is not a 0.25 second cycle");

        PreferredPoseShotSetup overlapping = new PreferredPoseShotSetup(
                new PreferredPoseShotSetup.ShotPose[] {
                        new PreferredPoseShotSetup.ShotPose(80.0, 0.0, Math.PI / 2.0)
                },
                40.0,
                Math.PI,
                0.80);
        double overlapped = overlapping.estimateSeconds(0.0, 0.0, 0.0, BallLoad.empty());
        near(overlapped, 2.80, 1e-9, "M translation and yaw overlap, so the shot uses the slower one");
        check(overlapped < 3.2, "M does not add translation and yaw");

        MapTipModel partial = new MapTipModel().set(0, 2, 0.30).set(0, 3, 0.90);
        PickupPlanner acquiring = planner(partial, new PreferredPoseShotSetup(
                new PreferredPoseShotSetup.ShotPose(80.0, 0.0, 0.0)));
        TrackedPiece besideShot = ball(5, BallType.NECTAR, 70.0, 0.0);
        PickupPlan grabbed = acquiring.plan(
                robot,
                new BallLoad(0, 2),
                Arrays.asList(besideShot),
                0.0,
                null);
        check(grabbed.getDecision() == PickupPlan.Decision.PICKUP, "M a nearby piece can beat shoot-now");
        check(containsId(grabbed, 5), "M collects the piece that finishes by the shot");
    }

    /** Eight-plus close pollen must not erase three farther nectar the tip model wants. */
    private static void testNectarSurvivesAPollenCrowd() {
        MapTipModel tips = new MapTipModel().set(0, 3, 0.95);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> pieces = new ArrayList<TrackedPiece>();
        for (int i = 0; i < 9; i++) {
            pieces.add(ball(i + 1, BallType.POLLEN, 6.0 + (4.0 * i), 0.0));
        }
        pieces.add(ball(21, BallType.NECTAR, 90.0, 0.0));
        pieces.add(ball(22, BallType.NECTAR, 108.0, 0.0));
        pieces.add(ball(23, BallType.NECTAR, 126.0, 0.0));

        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(plan.getTargets().size() == 3, "N takes three nectar");
        check(allType(plan, BallType.NECTAR), "N the pollen crowd does not hide the nectar");
    }

    /** The reserve is not nectar-biased. Far pollen survive a nectar crowd. */
    private static void testPollenSurvivesANectarCrowd() {
        MapTipModel tips = new MapTipModel().set(3, 0, 0.92);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> pieces = new ArrayList<TrackedPiece>();
        for (int i = 0; i < 9; i++) {
            pieces.add(ball(i + 1, BallType.NECTAR, 6.0 + (4.0 * i), 0.0));
        }
        pieces.add(ball(21, BallType.POLLEN, 90.0, 0.0));
        pieces.add(ball(22, BallType.POLLEN, 108.0, 0.0));
        pieces.add(ball(23, BallType.POLLEN, 126.0, 0.0));

        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(plan.getTargets().size() == 3, "O takes three pollen");
        check(allType(plan, BallType.POLLEN), "O the nectar crowd does not hide the pollen");
    }

    private static int[] fastestOrder(RobotState robot, List<TrackedPiece> pieces) {
        int[] order = new int[pieces.size()];
        int[] best = new int[pieces.size()];
        boolean[] used = new boolean[pieces.size()];
        double[] bestTime = new double[] {Double.POSITIVE_INFINITY};
        searchOrders(robot, pieces, order, used, 0, best, bestTime);
        return best;
    }

    private static void searchOrders(
            RobotState robot,
            List<TrackedPiece> pieces,
            int[] order,
            boolean[] used,
            int depth,
            int[] best,
            double[] bestTime) {
        if (depth == pieces.size()) {
            List<TrackedPiece> route = new ArrayList<TrackedPiece>();
            for (int i = 0; i < order.length; i++) {
                route.add(pieces.get(order[i]));
            }
            double time = routeSeconds(robot, route);
            int[] ids = new int[route.size()];
            for (int i = 0; i < route.size(); i++) {
                ids[i] = route.get(i).getId();
            }
            boolean betterTime = time < bestTime[0] - 1.0e-9;
            boolean tie = Math.abs(time - bestTime[0]) <= 1.0e-9 && lexicographicallyBefore(ids, best);
            if (betterTime || tie) {
                bestTime[0] = time;
                System.arraycopy(ids, 0, best, 0, ids.length);
            }
            return;
        }
        for (int i = 0; i < pieces.size(); i++) {
            if (used[i]) {
                continue;
            }
            used[i] = true;
            order[depth] = i;
            searchOrders(robot, pieces, order, used, depth + 1, best, bestTime);
            used[i] = false;
        }
    }

    private static boolean lexicographicallyBefore(int[] candidate, int[] incumbent) {
        for (int i = 0; i < candidate.length; i++) {
            if (candidate[i] != incumbent[i]) {
                return candidate[i] < incumbent[i];
            }
        }
        return false;
    }

    private static double routeSeconds(RobotState robot, List<TrackedPiece> ordered) {
        EuclideanTravelTimeModel travel = new EuclideanTravelTimeModel();
        double x = robot.getFieldX();
        double y = robot.getFieldY();
        double heading = robot.getHeadingRad();
        double time = 0.0;
        for (int i = 0; i < ordered.size(); i++) {
            PickupTarget target = CaptureGeometry.through(
                    ordered.get(i),
                    x,
                    y,
                    heading,
                    PickupPlannerConstants.CAPTURE_LEAD_INCHES);
            time += travel.estimateSeconds(x, y, heading, target);
            x = target.getCaptureX();
            y = target.getCaptureY();
            heading = target.getApproachHeadingRad();
        }
        return time;
    }

    private static List<TrackedPiece> piecesInOrder(List<TrackedPiece> pieces, int[] ids) {
        List<TrackedPiece> ordered = new ArrayList<TrackedPiece>();
        for (int i = 0; i < ids.length; i++) {
            for (int j = 0; j < pieces.size(); j++) {
                if (pieces.get(j).getId() == ids[i]) {
                    ordered.add(pieces.get(j));
                }
            }
        }
        return ordered;
    }

    private static PickupPlanner planner(MapTipModel tips) {
        return planner(tips, new FixedShotSetupModel(PickupPlannerConstants.SHOT_EXECUTION_SEC));
    }

    private static PickupPlanner planner(MapTipModel tips, ShotSetupTimeModel shots) {
        return new PickupPlanner(tips, new EuclideanTravelTimeModel(), shots);
    }

    private static double distanceToShot(PickupPlan plan, PreferredPoseShotSetup.ShotPose shot) {
        return Math.hypot(plan.getEndpointX() - shot.getX(), plan.getEndpointY() - shot.getY());
    }

    private static RobotState pose(double x, double y) {
        return new RobotState(
                0.0, x, y, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                true, true);
    }

    private static TrackedPiece ball(int id, BallType type, double x, double y) {
        return seen(id, type, x, y, 1.0);
    }

    private static TrackedPiece seen(int id, BallType type, double x, double y, double confidence) {
        return new TrackedPiece(id, type, PieceOwnership.NEUTRAL, x, y, confidence, 0.0, true);
    }

    private static int[] ids(PickupPlan plan) {
        int[] id = new int[plan.getTargets().size()];
        for (int i = 0; i < id.length; i++) {
            id[i] = plan.getTargets().get(i).getPiece().getId();
        }
        return id;
    }

    private static boolean containsId(PickupPlan plan, int id) {
        int[] found = ids(plan);
        for (int i = 0; i < found.length; i++) {
            if (found[i] == id) {
                return true;
            }
        }
        return false;
    }

    private static boolean allType(PickupPlan plan, BallType type) {
        return countType(plan, type) == plan.getTargets().size() && plan.getTargets().size() > 0;
    }

    private static int countType(PickupPlan plan, BallType type) {
        int count = 0;
        for (int i = 0; i < plan.getTargets().size(); i++) {
            if (plan.getTargets().get(i).getPiece().getType() == type) {
                count++;
            }
        }
        return count;
    }

    private static void testTravelTimeUsesTheLongerAxis() {
        EuclideanTravelTimeModel model = new EuclideanTravelTimeModel(40.0, Math.PI, 0.0);
        double translation = model.estimateSeconds(0.0, 0.0, 0.0, new PickupTarget(null, 80.0, 0.0, 0.0));
        near(translation, 2.0, 1.0e-9, "P 80 inches at 40 in/s is 2.0 s before intake overhead");

        double yaw = model.estimateSeconds(0.0, 0.0, 0.0, new PickupTarget(null, 0.0, 0.0, Math.PI / 2.0));
        near(yaw, 0.5, 1.0e-9, "Q a quarter turn at pi rad/s is 0.5 s");

        double together = model.estimateSeconds(0.0, 0.0, 0.0, new PickupTarget(null, 80.0, 0.0, Math.PI / 2.0));
        near(together, 2.0, 1.0e-9, "R simultaneous yaw does not add onto the translation");
        check(Math.abs(together - 2.5) > 0.1, "R the estimate is not translation plus yaw");

        EuclideanTravelTimeModel withIntake = new EuclideanTravelTimeModel(
                40.0, Math.PI, PickupPlannerConstants.ACQUISITION_OVERHEAD_SEC);
        near(withIntake.estimateSeconds(0.0, 0.0, 0.0, new PickupTarget(null, 80.0, 0.0, Math.PI / 2.0)),
                2.0 + PickupPlannerConstants.ACQUISITION_OVERHEAD_SEC,
                1.0e-9,
                "R intake overhead is added after the longer movement");
    }

    private static void near(double actual, double expected, double tolerance, String label) {
        checks++;
        if (!(Math.abs(actual - expected) <= tolerance)) {
            throw new AssertionError(label + " expected " + expected + " but was " + actual);
        }
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) {
            throw new AssertionError(label);
        }
    }
}
