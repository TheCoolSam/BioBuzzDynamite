package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Chance that a load tips the HIVE. The planner has no built-in rates.
 * A later model may also take shot geometry. V0 only passes the load.
 */
public interface TipModel {

    /**
     * Probability in [0, 1] that {@code load} produces a tip.
     * Callers should return a finite value. The planner treats a non-finite
     * result as zero.
     */
    double getTipProbability(BallLoad load);

    /** Distinguishes unavailable measurements from a measured zero probability. */
    default boolean hasEstimate(BallLoad load) { return Double.isFinite(getTipProbability(load)); }
}
