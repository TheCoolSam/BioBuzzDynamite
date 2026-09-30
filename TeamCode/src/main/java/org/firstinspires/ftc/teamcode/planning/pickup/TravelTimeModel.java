package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Seconds to go from a pose to one capture target, including the grab.
 * The planner chains this from target to target. A later Pedro-backed
 * implementation can replace the Euclidean estimate without a planner change.
 */
public interface TravelTimeModel {

    double estimateSeconds(
            double startX,
            double startY,
            double startHeadingRad,
            PickupTarget target);
}
