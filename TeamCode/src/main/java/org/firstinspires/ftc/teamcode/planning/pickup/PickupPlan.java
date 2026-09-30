package org.firstinspires.ftc.teamcode.planning.pickup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The route the planner wants, or an explicit decision to shoot without driving.
 * Targets are capture poses in field inches. A path follower consumes this
 * later. This object does not command motors.
 */
public final class PickupPlan {

    public enum Decision {
        /** Current load is the best use of time. Drive nowhere. */
        SHOOT_NOW,
        /** Drive the capture poses in order. */
        PICKUP,
        /** Pose is unusable. Do not move as if the robot were at the origin. */
        INVALID
    }

    private final Decision decision;
    private final List<PickupTarget> targets;
    private final BallLoad resultingLoad;
    private final double tipProbability;
    private final double estimatedSeconds;
    private final double planConfidence;
    private final double utility;

    private PickupPlan(
            Decision decision,
            List<PickupTarget> targets,
            BallLoad resultingLoad,
            double tipProbability,
            double estimatedSeconds,
            double planConfidence,
            double utility) {
        this.decision = decision;
        this.targets = Collections.unmodifiableList(new ArrayList<PickupTarget>(targets));
        this.resultingLoad = resultingLoad;
        this.tipProbability = tipProbability;
        this.estimatedSeconds = estimatedSeconds;
        this.planConfidence = planConfidence;
        this.utility = utility;
    }

    public static PickupPlan invalid(BallLoad load) {
        return new PickupPlan(
                Decision.INVALID,
                new ArrayList<PickupTarget>(),
                load == null ? BallLoad.empty() : load,
                0.0,
                0.0,
                0.0,
                0.0);
    }

    public static PickupPlan of(
            Decision decision,
            List<PickupTarget> targets,
            BallLoad resultingLoad,
            double tipProbability,
            double estimatedSeconds,
            double planConfidence,
            double utility) {
        return new PickupPlan(
                decision,
                targets,
                resultingLoad,
                tipProbability,
                estimatedSeconds,
                planConfidence,
                utility);
    }

    /**
     * Same route, different stored utility. Hysteresis compares this stored
     * number, so a committed plan does not get quietly re-scored into a switch.
     */
    public PickupPlan withUtility(double newUtility) {
        double stored = Double.isFinite(newUtility) ? newUtility : 0.0;
        return new PickupPlan(
                decision,
                targets,
                resultingLoad,
                tipProbability,
                estimatedSeconds,
                planConfidence,
                stored);
    }

    public Decision getDecision() {
        return decision;
    }

    public boolean isValid() {
        return decision != Decision.INVALID;
    }

    public List<PickupTarget> getTargets() {
        return targets;
    }

    public BallLoad getResultingLoad() {
        return resultingLoad;
    }

    public double getTipProbability() {
        return tipProbability;
    }

    public double getEstimatedSeconds() {
        return estimatedSeconds;
    }

    public double getPlanConfidence() {
        return planConfidence;
    }

    public double getUtility() {
        return utility;
    }
}
