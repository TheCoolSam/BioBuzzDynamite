package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.planning.pickup.BallLoad;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.CaptureGeometry;
import org.firstinspires.ftc.teamcode.planning.pickup.EuclideanTravelTimeModel;
import org.firstinspires.ftc.teamcode.planning.pickup.MapTipModel;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlan;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlanner;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlannerConstants;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupTarget;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
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
        testTargetDisappears();
        testInvalidPoseDoesNotUseOrigin();
        testCollectThenReplan();
        testCrowdedPoolDropsTheNinth();
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
        double plannedTime = plan.getEstimatedSeconds();
        check(plannedTime < listedTime - 1.0e-6, "D does not keep the input order");
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

    /** 3 percent is not enough to abandon the committed route. 25 percent is. */
    private static void testHysteresis() {
        MapTipModel tips = new MapTipModel().set(0, 2, 0.80);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> pieces = Arrays.asList(
                ball(1, BallType.NECTAR, 30.0, 0.0),
                ball(2, BallType.NECTAR, 48.0, 0.0));
        PickupPlan fresh = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, null);
        check(fresh.getUtility() > 0.0, "I has a positive utility");

        PickupPlan threePercent = fresh.withUtility(fresh.getUtility() / 1.03);
        PickupPlan kept = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, threePercent);
        check(kept == threePercent, "I keeps the plan when the new one is only 3 percent better");

        PickupPlan twentyFive = fresh.withUtility(fresh.getUtility() / 1.25);
        PickupPlan switched = planner.plan(pose(0.0, 0.0), BallLoad.empty(), pieces, 0.0, twentyFive);
        check(switched != twentyFive, "I switches when the new plan is 25 percent better");
        near(switched.getUtility(), fresh.getUtility(), 1e-9, "I switch adopts the new utility");
        check(Arrays.equals(ids(switched), ids(fresh)), "I switch adopts the new route");
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
     * Nine pieces exceed the pool. The far high-value nectar loses its seat to
     * eight nearer pollen. This is the cap, not the route score. Test A shows
     * a far nectar still wins when it fits in the pool.
     */
    private static void testCrowdedPoolDropsTheNinth() {
        MapTipModel tips = new MapTipModel().set(0, 1, 0.99);
        PickupPlanner planner = planner(tips);
        List<TrackedPiece> crowded = new ArrayList<TrackedPiece>();
        for (int i = 0; i < PickupPlannerConstants.MAX_CANDIDATES; i++) {
            crowded.add(ball(i + 1, BallType.POLLEN, 8.0 + (8.0 * i), 0.0));
        }
        crowded.add(ball(100, BallType.NECTAR, 220.0, 0.0));
        PickupPlan plan = planner.plan(pose(0.0, 0.0), BallLoad.empty(), crowded, 0.0, null);
        check(plan.getDecision() == PickupPlan.Decision.SHOOT_NOW, "pool cap drops the ninth piece");
        check(!containsId(plan, 100), "far nectar was not in the candidate pool");
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
        return new PickupPlanner(tips, new EuclideanTravelTimeModel());
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
