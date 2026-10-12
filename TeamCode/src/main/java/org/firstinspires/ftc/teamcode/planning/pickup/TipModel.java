package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Chance that a load tips the HIVE. The planner has no built-in rates.
 * Load-only estimates guide pickup planning; shot estimates require their own measurements.
 */
public interface TipModel {

    /**
     * Probability in [0, 1] that {@code load} produces a tip.
     * Callers should return a finite value. The planner treats a non-finite
     * result as zero.
     */
    double getTipProbability(BallLoad load);

    /** Override with measured shot/hive/cadence behavior. Unknown stays unknown. */
    default double getTipProbability(org.firstinspires.ftc.teamcode.scoring.ShotTipContext shot) {return Double.NaN;}

    /** Distinguishes unavailable measurements from a measured zero probability. */
    default boolean hasEstimate(BallLoad load) { return Double.isFinite(getTipProbability(load)); }
}
