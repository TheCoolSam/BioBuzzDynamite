package org.firstinspires.ftc.teamcode.state;

/**
 * Raw field motion from whatever localization is installed later.
 * The estimator turns these readings into a {@link RobotState}.
 *
 * <p>Distances are inches. Heading is radians in the
 * {@link org.firstinspires.ftc.teamcode.math.AngleUtil} frame.
 * This interface does not name a localization library. A future adapter
 * is what talks to that library and implements this.
 */
public interface RobotStateSource {

    /** Inches. */
    double getX();

    /** Inches. */
    double getY();

    double getHeadingRad();

    /** Inches per second, field frame. */
    double getFieldVx();

    /** Inches per second, field frame. */
    double getFieldVy();

    double getAngularVelocityRadPerSec();

    boolean isPoseValid();

    boolean isVelocityValid();

    /** Hardware sources must provide actual acquisition metadata. Missing health fails closed. */
    default double getAcquisitionTimestampSec() { return Double.NaN; }
    default boolean isDeviceHealthy() { return false; }
    default long getResetGeneration() { return 0; }
}
