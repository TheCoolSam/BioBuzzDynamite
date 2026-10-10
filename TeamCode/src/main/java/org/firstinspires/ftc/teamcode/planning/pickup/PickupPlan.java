package org.firstinspires.ftc.teamcode.planning.pickup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The route the planner wants, or an explicit decision to shoot without driving.
 * Targets are capture poses in field inches, with a selected shot setup when
 * the injected model supplies one. This object does not command motors.
 * SHOOT_NOW is strategic intent only. An executor may fire a feeder only
 * when SHOOT_NOW AND staged-piece-confirmed AND shooter-ready are all true.
 */
public final class PickupPlan {

    public enum Decision {
        /** No pickup targets. Time still includes getting from this pose to a shot. */
        SHOOT_NOW,
        /** Drive the capture poses in order, then set up the shot from the last one. */
        PICKUP,
        /** Valid input, but no feasible route with positive scoring value. Search/wait. */
        WAIT,
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
    private final double endpointX;
    private final double endpointY;
    private final double endpointHeadingRad;
    private ShotSetupPlan shotSetup;

    private PickupPlan(
            Decision decision,
            List<PickupTarget> targets,
            BallLoad resultingLoad,
            double tipProbability,
            double estimatedSeconds,
            double planConfidence,
            double utility,
            double endpointX,
            double endpointY,
            double endpointHeadingRad) {
        this.decision = decision;
        this.targets = Collections.unmodifiableList(new ArrayList<PickupTarget>(targets));
        this.resultingLoad = resultingLoad;
        this.tipProbability = tipProbability;
        this.estimatedSeconds = estimatedSeconds;
        this.planConfidence = planConfidence;
        this.utility = utility;
        this.endpointX = endpointX;
        this.endpointY = endpointY;
        this.endpointHeadingRad = endpointHeadingRad;
    }

    public static PickupPlan invalid(BallLoad load) {
        return new PickupPlan(
                Decision.INVALID,
                new ArrayList<PickupTarget>(),
                load == null ? BallLoad.empty() : load,
                0.0,
                0.0,
                0.0,
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
            double utility,
            double endpointX,
            double endpointY,
            double endpointHeadingRad) {
        return new PickupPlan(
                decision,
                targets,
                resultingLoad,
                tipProbability,
                estimatedSeconds,
                planConfidence,
                utility,
                endpointX,
                endpointY,
                endpointHeadingRad);
    }

    /**
     * Same route, different reported utility. The planner re-scores commitments
     * against current observations, so this value does not override hysteresis.
     */
    public PickupPlan withUtility(double newUtility) {
        double stored = Double.isFinite(newUtility) ? newUtility : 0.0;
        PickupPlan copy = new PickupPlan(
                decision,
                targets,
                resultingLoad,
                tipProbability,
                estimatedSeconds,
                planConfidence,
                stored,
                endpointX,
                endpointY,
                endpointHeadingRad);
        copy.shotSetup=shotSetup;
        return copy;
    }

    /** Return a copy carrying the selected scoring pose; scalar-only fixtures leave it absent. */
    public PickupPlan withShotSetup(ShotSetupPlan setup) {
        PickupPlan copy=withUtility(utility);copy.shotSetup=setup;return copy;
    }
    public ShotSetupPlan getShotSetup() {return shotSetup;}

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

    /**
     * Pose where shot setup starts. The robot's current pose for shoot-now,
     * or the last capture pose after a pickup route. Inches and radians.
     */
    public double getEndpointX() {
        return endpointX;
    }

    public double getEndpointY() {
        return endpointY;
    }

    public double getEndpointHeadingRad() {
        return endpointHeadingRad;
    }
}
