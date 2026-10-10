package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.state.RobotState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Chooses which pieces to collect, and in what order, to maximize expected
 * HIVE tip points per second. It does not drive.
 *
 * <p>Utility of a full route, not of the first ball:
 * <pre>
 *   G = geometric mean of the confidences on that route (1 when shooting now)
 *   planConfidence = (1 - CONFIDENCE_BLEND) + CONFIDENCE_BLEND * G
 *   totalTimeToTip = pickupTravelAndIntake + shotSetupAndExecution
 *   utility = (TIP_POINTS * tipProbability * planConfidence)
 *              / max(MINIMUM_CYCLE_SEC, totalTimeToTip)
 * </pre>
 * Shoot-now uses the same shot model from the robot's current pose, with no
 * pickup time. The minimum is only a floor so a zero estimate cannot make
 * utility infinite. The blend keeps an uncalibrated confidence score from
 * deleting an otherwise good route.
 *
 * <p>Search is every ordered sequence whose length is at most the remaining
 * capacity, drawn from at most {@link PickupPlannerConstants#MAX_CANDIDATES}
 * pieces. For n = 10 and depth 4 that is 5861 routes, each O(depth) to score.
 * Work is O(m log m) to prune plus O(depth * P(n, depth)). When more pieces
 * are visible, each ball type first keeps up to
 * {@link PickupPlannerConstants#RESERVED_CANDIDATES_PER_TYPE} of its best
 * nearby confident detections, round-robin so neither type fills the pool
 * alone. Leftover slots use {@code confidence / (1 + distance)}. That ranking
 * is only admission. Tip value is decided later, on complete routes.
 *
 * <p>A committed plan is kept until its route is impossible or a new route
 * beats its current utility by {@link PickupPlannerConstants#REPLAN_IMPROVEMENT_THRESHOLD}.
 * The committed target order is re-scored with fresh observations and the
 * current robot pose on every call; its old coordinates and costs are never reused.
 * Impossible means a listed piece is gone, stale, or illegal, the same piece
 * is listed twice, or the route no longer fits in the robot. After a successful
 * intake the caller adds the piece to {@link BallLoad}, removes it from the
 * visible list, and calls {@link #plan} again.
 * Pass null for the commitment after completion, cancellation, failure, or
 * a localization reset. Inventory changes also invalidate the old commitment.
 *
 * <p>A later path follower should drive {@link PickupPlan#getTargets()} in
 * order, then go from {@link PickupPlan#getEndpointX()} to a scoring pose.
 * Replace {@link TravelTimeModel} and {@link ShotSetupTimeModel} with
 * follower-aware estimates. This class must not gain a dependency on that follower.
 */
public final class PickupPlanner {

    private static final double TIE_EPSILON = 1.0e-9;

    private final TipModel tipModel;
    private final TravelTimeModel travel;
    private final ShotSetupTimeModel shotSetup;
    private final CaptureFeasibilityModel feasibility;

    /**
     * Nominal field center bounds only, with zero wall clearance because CAD is
     * unavailable. Before hardware execution inject measured robot clearance
     * using the four-argument constructor; zero is not a certified body footprint.
     */
    public PickupPlanner(TipModel tipModel, TravelTimeModel travel, ShotSetupTimeModel shotSetup) {
        this(tipModel, travel, shotSetup, new FieldBoundsCaptureFeasibility(
                PickupPlannerConstants.FIELD_WIDTH_INCHES, PickupPlannerConstants.FIELD_HEIGHT_INCHES, 0, 0));
    }

    public PickupPlanner(TipModel tipModel, TravelTimeModel travel, ShotSetupTimeModel shotSetup,
            CaptureFeasibilityModel feasibility) {
        this.tipModel = tipModel;
        this.travel = travel;
        this.shotSetup = shotSetup;
        if (feasibility == null) throw new IllegalArgumentException("Capture feasibility model required");
        this.feasibility = feasibility;
    }

    /**
     * @param committed previous plan to stick with, or null when nothing is committed
     */
    public PickupPlan plan(
            RobotState robot,
            BallLoad onboard,
            List<TrackedPiece> visible,
            double nowSec,
            PickupPlan committed) {
        BallLoad held = onboard == null ? BallLoad.empty() : onboard;
        if (robot == null || !robot.isFresh(nowSec, PickupPlannerConstants.MAX_POSE_AGE_SEC)
                || held.total() > PickupPlannerConstants.MAX_CAPACITY) {
            return PickupPlan.invalid(held);
        }

        List<TrackedPiece> candidates = selectCandidates(visible, nowSec);
        int remaining = held.remainingCapacity(PickupPlannerConstants.MAX_CAPACITY);
        List<TrackedPiece> pool = candidates.size() > PickupPlannerConstants.MAX_CANDIDATES
                ? capPool(candidates, robot.getFieldX(), robot.getFieldY()) : candidates;
        PickupPlan best = search(robot, held, pool, remaining);
        // Validate the incumbent against every fresh piece, not just the
        // admission pool. Pool pruning is not evidence that a target vanished.
        return applyHysteresis(best, committed, robot, held, candidates, remaining);
    }

    private List<TrackedPiece> selectCandidates(List<TrackedPiece> visible, double nowSec) {
        List<TrackedPiece> fresh = new ArrayList<TrackedPiece>();
        if (visible == null) {
            return fresh;
        }
        for (int i = 0; i < visible.size(); i++) {
            TrackedPiece piece = visible.get(i);
            if (!usable(piece, nowSec)) {
                continue;
            }
            int existing = indexOfId(fresh, piece.getId());
            if (existing < 0) {
                fresh.add(piece);
            } else if (prefer(piece, fresh.get(existing))) {
                fresh.set(existing, piece);
            }
        }
        return fresh;
    }

    /**
     * Keep a reserved slice of every ball type, then fill what is left.
     * Round-robin across types so pollen cannot consume the reserve before
     * nectar is considered, or the other way around.
     */
    private static List<TrackedPiece> capPool(List<TrackedPiece> fresh, final double originX, final double originY) {
        BallType[] types = BallType.values();
        List<List<TrackedPiece>> groups = new ArrayList<List<TrackedPiece>>();
        for (int typeIndex = 0; typeIndex < types.length; typeIndex++) {
            groups.add(new ArrayList<TrackedPiece>());
        }
        for (int i = 0; i < fresh.size(); i++) {
            TrackedPiece piece = fresh.get(i);
            groups.get(typeIndex(piece.getType(), types)).add(piece);
        }
        for (int typeIndex = 0; typeIndex < groups.size(); typeIndex++) {
            sortByPoolScore(groups.get(typeIndex), originX, originY);
        }

        List<TrackedPiece> pool = new ArrayList<TrackedPiece>();
        int[] cursor = new int[types.length];
        int reserve = PickupPlannerConstants.RESERVED_CANDIDATES_PER_TYPE;
        for (int round = 0; round < reserve && pool.size() < PickupPlannerConstants.MAX_CANDIDATES; round++) {
            for (int typeIndex = 0; typeIndex < types.length
                    && pool.size() < PickupPlannerConstants.MAX_CANDIDATES; typeIndex++) {
                List<TrackedPiece> group = groups.get(typeIndex);
                if (cursor[typeIndex] < group.size()) {
                    pool.add(group.get(cursor[typeIndex]));
                    cursor[typeIndex]++;
                }
            }
        }

        List<TrackedPiece> rest = new ArrayList<TrackedPiece>();
        for (int typeIndex = 0; typeIndex < groups.size(); typeIndex++) {
            List<TrackedPiece> group = groups.get(typeIndex);
            for (int i = cursor[typeIndex]; i < group.size(); i++) {
                rest.add(group.get(i));
            }
        }
        sortByPoolScore(rest, originX, originY);
        for (int i = 0; i < rest.size() && pool.size() < PickupPlannerConstants.MAX_CANDIDATES; i++) {
            pool.add(rest.get(i));
        }
        return pool;
    }

    private static int typeIndex(BallType type, BallType[] types) {
        for (int i = 0; i < types.length; i++) {
            if (types[i] == type) {
                return i;
            }
        }
        return 0;
    }

    private static void sortByPoolScore(List<TrackedPiece> pieces, final double originX, final double originY) {
        Collections.sort(pieces, new Comparator<TrackedPiece>() {
            @Override
            public int compare(TrackedPiece a, TrackedPiece b) {
                int byScore = Double.compare(poolScore(b, originX, originY), poolScore(a, originX, originY));
                if (byScore != 0) {
                    return byScore;
                }
                return Integer.compare(a.getId(), b.getId());
            }
        });
    }

    /**
     * Admission ranking only. Near and well-seen pieces are preferred when a
     * type has more detections than its reserve. Tip value is not in this score.
     */
    private static double poolScore(TrackedPiece piece, double originX, double originY) {
        double dx = piece.getFieldX() - originX;
        double dy = piece.getFieldY() - originY;
        double distance = Math.hypot(dx, dy);
        return clamp01(piece.getConfidence()) / (1.0 + distance);
    }

    private static boolean prefer(TrackedPiece candidate, TrackedPiece incumbent) {
        if (candidate.getLastSeenTimestampSec() > incumbent.getLastSeenTimestampSec()) {
            return true;
        }
        if (candidate.getLastSeenTimestampSec() < incumbent.getLastSeenTimestampSec()) {
            return false;
        }
        return candidate.getConfidence() > incumbent.getConfidence();
    }

    private static boolean usable(TrackedPiece piece, double nowSec) {
        if (piece == null || !piece.isCollectable() || piece.getType() == null) {
            return false;
        }
        if (!Double.isFinite(piece.getFieldX())
                || !Double.isFinite(piece.getFieldY())
                || !Double.isFinite(piece.getConfidence())) {
            return false;
        }
        double age = nowSec - piece.getLastSeenTimestampSec();
        return Double.isFinite(age) && age >= 0.0
                && age <= PickupPlannerConstants.MAX_OBSERVATION_AGE_SEC;
    }

    private PickupPlan search(
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int remaining) {
        PickupPlan best = score(robot, held, candidates, new int[0]);
        if (best == null) best = waitPlan(robot, held);
        if (remaining == 0 || candidates.isEmpty()) {
            return best;
        }
        int depth = Math.min(remaining, candidates.size());
        int[] order = new int[depth];
        boolean[] used = new boolean[candidates.size()];
        best = permute(best, robot, held, candidates, order, used, 0, depth);
        return best;
    }

    private PickupPlan permute(
            PickupPlan best,
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int[] order,
            boolean[] used,
            int depth,
            int maxDepth) {
        PickupPlan here = score(robot, held, candidates, order, depth);
        if (better(here, best)) {
            best = here;
        }
        if (depth == maxDepth) {
            return best;
        }
        for (int i = 0; i < candidates.size(); i++) {
            if (used[i]) {
                continue;
            }
            used[i] = true;
            order[depth] = i;
            best = permute(best, robot, held, candidates, order, used, depth + 1, maxDepth);
            used[i] = false;
        }
        return best;
    }

    private PickupPlan score(
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int[] order) {
        return score(robot, held, candidates, order, 0);
    }

    private PickupPlan score(
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int[] order,
            int length) {
        double x = robot.getFieldX();
        double y = robot.getFieldY();
        double heading = robot.getHeadingRad();
        double time = 0.0;
        double confidenceProduct = 1.0;
        BallLoad load = held;
        List<PickupTarget> targets = new ArrayList<PickupTarget>();

        for (int i = 0; i < length; i++) {
            TrackedPiece piece = candidates.get(order[i]);
            PickupTarget target = CaptureGeometry.feasibleThrough(
                    piece,
                    x,
                    y,
                    heading,
                    PickupPlannerConstants.CAPTURE_LEAD_INCHES,
                    feasibility);
            if (target == null) return null; // Hard rejection, never a large cost penalty.
            double leg = travelSeconds(x, y, heading, target);
            if (!Double.isFinite(leg) || leg < 0) return null;
            time += leg;
            targets.add(target);
            load = load.plus(piece.getType());
            confidenceProduct *= clamp01(piece.getConfidence());
            x = target.getCaptureX();
            y = target.getCaptureY();
            heading = target.getApproachHeadingRad();
        }

        double planConfidence = length == 0
                ? 1.0
                : (1.0 - PickupPlannerConstants.CONFIDENCE_BLEND)
                + (PickupPlannerConstants.CONFIDENCE_BLEND * Math.pow(confidenceProduct, 1.0 / length));
        // x, y, heading are the robot now when nothing was picked up, otherwise
        // the last capture pose. Shot time is measured from that endpoint.
        double total = time + shotSeconds(x, y, heading, load);
        if (!Double.isFinite(total) || total < 0) return null;
        double cycle = Math.max(PickupPlannerConstants.MINIMUM_CYCLE_SEC, total);
        double tip = tipProbability(load);
        double utility = (PickupPlannerConstants.TIP_POINTS * tip * planConfidence) / cycle;
        if (load.total() == 0 || tip <= 0.0 || !Double.isFinite(utility) || utility <= 0.0) {
            if (length != 0) return null; // A longer route may still create a useful load.
            return waitPlan(robot, held);
        }
        PickupPlan.Decision decision = length == 0
                ? PickupPlan.Decision.SHOOT_NOW
                : PickupPlan.Decision.PICKUP;
        return PickupPlan.of(decision, targets, load, tip, total, planConfidence, utility, x, y, heading)
                .withShotSetup(shotSetup.select(x,y,heading,load));
    }

    private double shotSeconds(double x, double y, double heading, BallLoad load) {
        if (shotSetup == null) {
            return Double.NaN;
        }
        double seconds = shotSetup.estimateSeconds(x, y, heading, load);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            return Double.NaN;
        }
        return seconds;
    }

    private double travelSeconds(double x, double y, double heading, PickupTarget target) {
        if (travel == null) {
            return Double.NaN;
        }
        double seconds = travel.estimateSeconds(x, y, heading, target);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            return Double.NaN;
        }
        return seconds;
    }

    private double tipProbability(BallLoad load) {
        if (tipModel == null || !tipModel.hasEstimate(load)) {
            return 0.0;
        }
        double probability = tipModel.getTipProbability(load);
        if (!Double.isFinite(probability) || probability <= 0.0) {
            return 0.0;
        }
        if (probability >= 1.0) {
            return 1.0;
        }
        return probability;
    }

    private static PickupPlan waitPlan(RobotState robot, BallLoad held) {
        return PickupPlan.of(PickupPlan.Decision.WAIT, Collections.<PickupTarget>emptyList(), held,
                0, 0, 1, 0, robot.getFieldX(), robot.getFieldY(), robot.getHeadingRad());
    }

    private static boolean better(PickupPlan candidate, PickupPlan incumbent) {
        if (candidate == null) return false;
        if (candidate.getUtility() > incumbent.getUtility() + TIE_EPSILON) {
            return true;
        }
        if (incumbent.getUtility() > candidate.getUtility() + TIE_EPSILON) {
            return false;
        }
        if (candidate.getEstimatedSeconds() < incumbent.getEstimatedSeconds() - TIE_EPSILON) {
            return true;
        }
        if (incumbent.getEstimatedSeconds() < candidate.getEstimatedSeconds() - TIE_EPSILON) {
            return false;
        }
        return compareIds(candidate.getTargets(), incumbent.getTargets()) < 0;
    }

    private static int compareIds(List<PickupTarget> left, List<PickupTarget> right) {
        int n = Math.min(left.size(), right.size());
        for (int i = 0; i < n; i++) {
            int cmp = Integer.compare(left.get(i).getPiece().getId(), right.get(i).getPiece().getId());
            if (cmp != 0) {
                return cmp;
            }
        }
        return Integer.compare(left.size(), right.size());
    }

    private PickupPlan applyHysteresis(
            PickupPlan best,
            PickupPlan committed,
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int remaining) {
        int[] order = committedOrder(committed, held, candidates, remaining);
        if (order == null) {
            return best;
        }
        PickupPlan refreshed = score(robot, held, candidates, order, order.length);
        if (refreshed == null) return best; // Includes newly infeasible committed approaches.
        double required = refreshed.getUtility() * (1.0 + PickupPlannerConstants.REPLAN_IMPROVEMENT_THRESHOLD);
        if (best.getUtility() > required + TIE_EPSILON) {
            return best;
        }
        return refreshed;
    }

    /** Reconstruct the old order and check the inventory it was planned from. */
    private static int[] committedOrder(
            PickupPlan committed, BallLoad held, List<TrackedPiece> candidates, int remaining) {
        if (committed == null || !committed.isValid() || committed.getResultingLoad() == null) {
            return null;
        }
        List<PickupTarget> targets = committed.getTargets();
        if (committed.getDecision() == PickupPlan.Decision.SHOOT_NOW) {
            if (!targets.isEmpty()) {
                return null;
            }
        } else if (committed.getDecision() != PickupPlan.Decision.PICKUP || targets.isEmpty()) {
            return null;
        }
        if (targets.size() > remaining) {
            return null;
        }
        int pollen = committed.getResultingLoad().getPollen();
        int nectar = committed.getResultingLoad().getNectar();
        int[] order = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PickupTarget target = targets.get(i);
            if (target == null || target.getPiece() == null) {
                return null;
            }
            int id = target.getPiece().getId();
            order[i] = indexOfId(candidates, id);
            if (order[i] < 0
                    || candidates.get(order[i]).getType() != target.getPiece().getType()) {
                return null;
            }
            for (int j = 0; j < i; j++) {
                if (targets.get(j).getPiece().getId() == id) {
                    return null;
                }
            }
            if (target.getPiece().getType() == BallType.NECTAR) {
                nectar--;
            } else {
                pollen--;
            }
        }
        return pollen == held.getPollen() && nectar == held.getNectar() ? order : null;
    }

    private static int indexOfId(List<TrackedPiece> pieces, int id) {
        for (int i = 0; i < pieces.size(); i++) {
            if (pieces.get(i).getId() == id) {
                return i;
            }
        }
        return -1;
    }

    private static double clamp01(double confidence) {
        if (!Double.isFinite(confidence) || confidence <= 0.0) {
            return 0.0;
        }
        if (confidence >= 1.0) {
            return 1.0;
        }
        return confidence;
    }
}
