package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Hard feasibility gate, evaluated before travel cost. Start is the current
 * pose or the preceding route endpoint, in field inches/radians. Implementations
 * may later check robot/intake footprints, HIVE legs, FLOWERS, and other obstacles.
 * This contract contains no follower or hardware dependency.
 */
public interface CaptureFeasibilityModel {
    boolean isFeasible(double startX, double startY, double startHeadingRad, PickupTarget target);
}
