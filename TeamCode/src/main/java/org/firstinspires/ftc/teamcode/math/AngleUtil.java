package org.firstinspires.ftc.teamcode.math;

/**
 * Coordinate convention for this robot. This is the only place that defines it.
 *
 * <ul>
 *   <li>+X is field right</li>
 *   <li>+Y is field forward</li>
 *   <li>heading 0 points along +X</li>
 *   <li>positive heading is counterclockwise</li>
 *   <li>positive turret yaw is counterclockwise relative to the chassis</li>
 *   <li>every angle inside the control code is radians</li>
 * </ul>
 *
 * Localization code (odometry, AprilTag, Road Runner, Pedro) often uses a different
 * field frame. Convert into this frame before calling the turret controller.
 * {@link #toDegrees(double)} is only for telemetry.
 */
public final class AngleUtil {

    /** One full turn, in radians. */
    public static final double TAU = 2.0 * Math.PI;

    private static final double RANGE_EPSILON = 1.0e-9;

    private AngleUtil() {
    }

    /**
     * Wraps an angle into [-π, π).
     * NaN and infinities stay NaN so callers can reject them.
     */
    public static double wrapRadians(double angleRad) {
        if (!Double.isFinite(angleRad)) {
            return Double.NaN;
        }

        // floor form stays accurate for a large accumulated heading, and folds +π to -π
        // so the result is [-π, π) rather than (-π, π].
        double wrapped = angleRad - (TAU * Math.floor((angleRad + Math.PI) / TAU));
        if (wrapped >= Math.PI) {
            wrapped -= TAU;
        }
        return wrapped;
    }

    /**
     * Field bearing from a robot position to a target.
     * If the target is on top of the robot the direction is undefined and this returns
     * {@code robotHeadingRad}, which makes the robot-relative turret angle zero.
     */
    public static double fieldBearing(
            double robotX,
            double robotY,
            double robotHeadingRad,
            double targetX,
            double targetY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        if (dx * dx + dy * dy < 1.0e-12) {
            return robotHeadingRad;
        }
        return Math.atan2(dy, dx);
    }

    /**
     * Robot-relative turret angle that points along {@code fieldBearingRad}.
     * Result is wrapped to [-π, π). This is the only copy of
     * {@code wrap(bearing - heading)}.
     */
    public static double robotRelativeAngle(double fieldBearingRad, double robotHeadingRad) {
        return wrapRadians(fieldBearingRad - robotHeadingRad);
    }

    /**
     * How fast the field bearing to a fixed target is changing, in rad/s.
     * {@code dx} and {@code dy} are target minus robot. Robot velocity is field-frame.
     * Returns 0 when the target is too close for that rate to be meaningful.
     *
     * <p>{@code (dy * robotVx - dx * robotVy) / (dx² + dy²)}
     */
    public static double bearingRate(double dx, double dy, double robotVx, double robotVy) {
        double radiusSquared = (dx * dx) + (dy * dy);
        if (!(radiusSquared > 1.0e-8)
                || !Double.isFinite(robotVx)
                || !Double.isFinite(robotVy)) {
            return 0.0;
        }
        double rate = ((dy * robotVx) - (dx * robotVy)) / radiusSquared;
        if (!Double.isFinite(rate)) {
            return 0.0;
        }
        return rate;
    }

    /**
     * Among angles equal to {@code angleRad + 2πk}, picks the one inside
     * [{@code minRad}, {@code maxRad}] closest to {@code currentRad}.
     * At most a few candidates exist for a turret with less than one full turn of travel.
     * A tie prefers the candidate closer to zero so the turret settles toward center.
     */
    public static EquivalentAngle closestEquivalentInInterval(
            double angleRad,
            double currentRad,
            double minRad,
            double maxRad) {
        if (!Double.isFinite(angleRad) || !Double.isFinite(minRad) || !Double.isFinite(maxRad)) {
            return EquivalentAngle.none();
        }
        if (!Double.isFinite(currentRad)) {
            currentRad = 0.0;
        }
        if (minRad > maxRad) {
            double swap = minRad;
            minRad = maxRad;
            maxRad = swap;
        }

        double wrapped = wrapRadians(angleRad);
        int kStart = (int) Math.floor((minRad - wrapped) / TAU) - 1;
        int kEnd = (int) Math.ceil((maxRad - wrapped) / TAU) + 1;
        if (kEnd - kStart > 32) {
            kStart = -3;
            kEnd = 3;
        }

        boolean found = false;
        double bestAngle = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int k = kStart; k <= kEnd; k++) {
            double candidate = wrapped + (k * TAU);
            if (candidate < minRad - RANGE_EPSILON || candidate > maxRad + RANGE_EPSILON) {
                continue;
            }
            candidate = clamp(candidate, minRad, maxRad);

            double distance = Math.abs(candidate - currentRad);
            boolean closer = distance < bestDistance - 1.0e-9;
            boolean tiePreferCenter = Math.abs(distance - bestDistance) <= 1.0e-9
                    && Math.abs(candidate) < Math.abs(bestAngle);
            if (!found || closer || tiePreferCenter) {
                found = true;
                bestAngle = candidate;
                bestDistance = distance;
            }
        }

        if (!found) {
            return EquivalentAngle.none();
        }
        return new EquivalentAngle(true, bestAngle);
    }

    public static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    /** Telemetry only. Control math must stay in radians. */
    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /** One equivalent angle inside a closed interval, or none. */
    public static final class EquivalentAngle {
        public final boolean reachable;
        public final double angleRad;

        EquivalentAngle(boolean reachable, double angleRad) {
            this.reachable = reachable;
            this.angleRad = angleRad;
        }

        static EquivalentAngle none() {
            return new EquivalentAngle(false, 0.0);
        }
    }
}
