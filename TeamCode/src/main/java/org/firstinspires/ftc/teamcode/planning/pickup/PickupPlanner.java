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
 *   cycleTime = max(MINIMUM_CYCLE_SEC, travel + intake time)
 *   utility = (TIP_POINTS * tipProbability * planConfidence) / cycleTime
 * </pre>
 * Dividing by time is what stops a long drive for a tiny probability gain.
 * Shoot-now is a real candidate: its cycle time is just
 * {@link PickupPlannerConstants#MINIMUM_CYCLE_SEC}. The blend keeps an
 * uncalibrated confidence score from deleting an otherwise good route.
 *
 * <p>Search is every ordered sequence whose length is at most the remaining
 * capacity, drawn from at most {@link PickupPlannerConstants#MAX_CANDIDATES}
 * pieces. For n = 8 and depth 4 that is 2081 routes, each O(depth) to score.
 * Work is O(n log n) to prune plus O(depth * P(n, depth)). When more than
 * 8 pieces are visible, the pool keeps the highest
 * {@code confidence / (1 + distance)} scores. That cap can drop a far but
 * valuable piece on a crowded field. Inside the pool the search is not a
 * nearest-ball rule.
 *
 * <p>A committed plan is kept until its route is impossible or a new route
 * beats its stored utility by {@link PickupPlannerConstants#REPLAN_IMPROVEMENT_THRESHOLD}.
 * Impossible means a listed piece is gone, stale, or illegal, the same piece
 * is listed twice, or the route no longer fits in the robot. After a successful
 * intake the caller adds the piece to {@link BallLoad}, removes it from the
 * visible list, and calls {@link #plan} again.
 */
public final class PickupPlanner {

    private static final double TIE_EPSILON = 1.0e-9;

    private final TipModel tipModel;
    private final TravelTimeModel travel;

    public PickupPlanner(TipModel tipModel, TravelTimeModel travel) {
        this.tipModel = tipModel;
        this.travel = travel;
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
        if (robot == null || !robot.isPoseValid()) {
            return PickupPlan.invalid(held);
        }

        List<TrackedPiece> candidates = selectCandidates(robot, visible, nowSec);
        int remaining = held.remainingCapacity(PickupPlannerConstants.MAX_CAPACITY);
        PickupPlan best = search(robot, held, candidates, remaining);
        return applyHysteresis(best, committed, candidates, remaining);
    }

    private List<TrackedPiece> selectCandidates(RobotState robot, List<TrackedPiece> visible, double nowSec) {
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
        if (fresh.size() <= PickupPlannerConstants.MAX_CANDIDATES) {
            return fresh;
        }

        final double originX = robot.getFieldX();
        final double originY = robot.getFieldY();
        Collections.sort(fresh, new Comparator<TrackedPiece>() {
            @Override
            public int compare(TrackedPiece a, TrackedPiece b) {
                int byScore = Double.compare(poolScore(b, originX, originY), poolScore(a, originX, originY));
                if (byScore != 0) {
                    return byScore;
                }
                return Integer.compare(a.getId(), b.getId());
            }
        });
        return new ArrayList<TrackedPiece>(fresh.subList(0, PickupPlannerConstants.MAX_CANDIDATES));
    }

    /**
     * Pool ranking only. Near and well-seen pieces stay when the field is
     * crowded. Tip value is decided later, on complete routes.
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
        return Double.isFinite(age) && age <= PickupPlannerConstants.MAX_OBSERVATION_AGE_SEC;
    }

    private PickupPlan search(
            RobotState robot,
            BallLoad held,
            List<TrackedPiece> candidates,
            int remaining) {
        PickupPlan best = score(robot, held, candidates, new int[0]);
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
            PickupTarget target = CaptureGeometry.through(
                    piece,
                    x,
                    y,
                    heading,
                    PickupPlannerConstants.CAPTURE_LEAD_INCHES);
            time += travelSeconds(x, y, heading, target);
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
        if (length == 0) {
            time = PickupPlannerConstants.MINIMUM_CYCLE_SEC;
        }
        double cycle = Math.max(PickupPlannerConstants.MINIMUM_CYCLE_SEC, time);
        double tip = tipProbability(load);
        double utility = (PickupPlannerConstants.TIP_POINTS * tip * planConfidence) / cycle;
        PickupPlan.Decision decision = length == 0
                ? PickupPlan.Decision.SHOOT_NOW
                : PickupPlan.Decision.PICKUP;
        return PickupPlan.of(decision, targets, load, tip, time, planConfidence, utility);
    }

    private double travelSeconds(double x, double y, double heading, PickupTarget target) {
        if (travel == null) {
            return 1.0e6;
        }
        double seconds = travel.estimateSeconds(x, y, heading, target);
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            return 1.0e6;
        }
        return seconds;
    }

    private double tipProbability(BallLoad load) {
        if (tipModel == null) {
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

    private static boolean better(PickupPlan candidate, PickupPlan incumbent) {
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

    private static PickupPlan applyHysteresis(
            PickupPlan best,
            PickupPlan committed,
            List<TrackedPiece> candidates,
            int remaining) {
        if (!routeStillOpen(committed, candidates, remaining)) {
            return best;
        }
        double required = committed.getUtility() * (1.0 + PickupPlannerConstants.REPLAN_IMPROVEMENT_THRESHOLD);
        if (best.getUtility() > required + TIE_EPSILON) {
            return best;
        }
        return committed;
    }

    private static boolean routeStillOpen(PickupPlan committed, List<TrackedPiece> candidates, int remaining) {
        if (committed == null || !committed.isValid()) {
            return false;
        }
        List<PickupTarget> targets = committed.getTargets();
        if (committed.getDecision() == PickupPlan.Decision.SHOOT_NOW) {
            return targets.isEmpty();
        }
        if (committed.getDecision() != PickupPlan.Decision.PICKUP || targets.isEmpty()) {
            return false;
        }
        if (targets.size() > remaining) {
            return false;
        }
        for (int i = 0; i < targets.size(); i++) {
            PickupTarget target = targets.get(i);
            if (target == null || target.getPiece() == null) {
                return false;
            }
            int id = target.getPiece().getId();
            if (indexOfId(candidates, id) < 0) {
                return false;
            }
            for (int j = 0; j < i; j++) {
                if (targets.get(j).getPiece().getId() == id) {
                    return false;
                }
            }
        }
        return true;
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
