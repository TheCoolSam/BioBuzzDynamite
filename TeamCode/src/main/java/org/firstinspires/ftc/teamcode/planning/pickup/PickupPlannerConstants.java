package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Placeholder knobs for the pickup planner. None of these are measured
 * HIVE tip rates. Tip chances come from a {@link TipModel} supplied at
 * runtime. Replace the travel numbers after the real robot is timed.
 */
public final class PickupPlannerConstants {

    /** Pieces the robot can hold at once. Placeholder until the intake is built. */
    public static final int MAX_CAPACITY = 4;

    /**
     * Largest set of pieces that enters the permutation search.
     * Ordered routes of length up to {@link #MAX_CAPACITY} from 8 pieces is
     * 2081 sequences. That stays cheap beside vision and the turret loop.
     * A 10-piece pool would still be fine, but 8 leaves margin and is already
     * more balls than this robot expects to see in one frame.
     */
    public static final int MAX_CANDIDATES = 8;

    /** Points awarded for a HIVE tip. The utility numerator uses this. */
    public static final double TIP_POINTS = 20.0;

    /**
     * A new route replaces the committed one only when its utility is at least
     * this much better. 0.15 means 15 percent.
     */
    public static final double REPLAN_IMPROVEMENT_THRESHOLD = 0.15;

    /**
     * Seconds charged to a plan that does not drive, including shoot-now.
     * Keeps the utility finite. Placeholder for "already aimed, just shoot".
     */
    public static final double MINIMUM_CYCLE_SEC = 0.25;

    /** Planning speed, inches per second. Not a motor characterization. */
    public static final double NOMINAL_SPEED_IN_PER_SEC = 40.0;

    /** Planning yaw rate, radians per second. About 180 deg/s. */
    public static final double YAW_RATE_RAD_PER_SEC = Math.PI;

    /** Extra seconds charged once per piece for the intake to finish the grab. */
    public static final double ACQUISITION_OVERHEAD_SEC = 0.30;

    /**
     * Inches past the piece where the capture pose sits, along the approach.
     * The robot is planned through the ball, not stopped on its center.
     */
    public static final double CAPTURE_LEAD_INCHES = 6.0;

    /**
     * Drop a detection older than this. Placeholder. A future tracker can
     * coast a piece longer if it wants the planner to keep it.
     */
    public static final double MAX_OBSERVATION_AGE_SEC = 0.50;

    /**
     * How hard detection confidence scales utility. 0 ignores confidence.
     * 1 would use the geometric mean directly. 0.40 keeps a poorly seen route
     * in the running, because these confidences will not be calibrated probabilities.
     */
    public static final double CONFIDENCE_BLEND = 0.40;

    private PickupPlannerConstants() {
    }
}
