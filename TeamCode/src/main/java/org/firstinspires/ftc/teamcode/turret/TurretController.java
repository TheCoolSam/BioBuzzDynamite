package org.firstinspires.ftc.teamcode.turret;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/**
 * Field-relative turret tracker.
 *
 * <p>Call {@link #calculate(TurretState)} once per loop from the OpMode thread.
 * It does not sleep, start threads, or talk to hardware. Motor power is applied
 * by the caller through {@link TurretIO}. Chassis omega is only a request.
 *
 * <p>The derivative term remembers the previous error, so construct one controller
 * and reuse it. Call {@link #reset()} if the turret angle is teleported in a test.
 */
public class TurretController {

    /** Ignore the D term when dt is tiny or after a long hitch. */
    private static final double MIN_DT_FOR_DERIVATIVE = 1.0e-4;
    private static final double MAX_DT_FOR_DERIVATIVE = 0.2;

    private double lastErrorRad = 0.0;
    private boolean hasLastError = false;

    public void reset() {
        lastErrorRad = 0.0;
        hasLastError = false;
    }

    public TurretCommand calculate(TurretState state) {
        if (!inputsUsable(state)) {
            return safeHold(state);
        }

        double bearing = AngleUtil.fieldBearing(
                state.robotX,
                state.robotY,
                state.robotHeading,
                state.targetX,
                state.targetY);
        double ideal = AngleUtil.robotRelativeAngle(bearing, state.robotHeading);

        AngleChoice choice = chooseReachableAngle(ideal, state.turretAngle);
        double desired = AngleUtil.clamp(
                choice.angleRad,
                TurretConstants.PHYSICAL_MIN_RAD,
                TurretConstants.PHYSICAL_MAX_RAD);

        // Do not wrap this error. The short wrap can point through the rear deadzone.
        // Both angles already lie inside the mechanical travel, so their difference
        // is the path that stays between the stops.
        double error = desired - state.turretAngle;

        double errorRate = 0.0;
        if (hasLastError
                && state.dt > MIN_DT_FOR_DERIVATIVE
                && state.dt <= MAX_DT_FOR_DERIVATIVE) {
            errorRate = (error - lastErrorRad) / state.dt;
        }
        lastErrorRad = error;
        hasLastError = true;

        // Counter-rotate the turret when the chassis is spun, including a collision.
        // Translation changes the bearing too; position feedback covers that for now.
        double desiredVelocity = -state.robotAngularVelocity;

        double power = (TurretConstants.KP * error)
                + (TurretConstants.KD * errorRate)
                + (TurretConstants.KV * desiredVelocity);
        if (!Double.isFinite(power)) {
            power = 0.0;
        }
        power = AngleUtil.clamp(
                power,
                TurretConstants.MIN_MOTOR_POWER,
                TurretConstants.MAX_MOTOR_POWER);
        power = blockPowerIntoStop(state.turretAngle, power);

        double chassisOmega = requestedChassisOmega(desired);

        return new TurretCommand(
                bearing,
                desired,
                error,
                desiredVelocity,
                power,
                chassisOmega,
                choice.reachable,
                Math.abs(desired) > TurretConstants.UNWIND_START_RAD);
    }

    /**
     * Prefer a 2π-equivalent aim inside the physical travel, closest to where
     * the turret already is. If the whole family sits in the rear deadzone,
     * hold the nearer physical stop instead of commanding through it.
     */
    private static AngleChoice chooseReachableAngle(double idealWrapped, double currentAngle) {
        AngleUtil.EquivalentAngle match = AngleUtil.closestEquivalentInInterval(
                idealWrapped,
                currentAngle,
                TurretConstants.PHYSICAL_MIN_RAD,
                TurretConstants.PHYSICAL_MAX_RAD);
        if (match.reachable) {
            return new AngleChoice(match.angleRad, true);
        }
        return new AngleChoice(nearestStop(idealWrapped), false);
    }

    /** Closer physical stop, measured the short way around the circle. */
    private static double nearestStop(double idealWrapped) {
        double min = TurretConstants.PHYSICAL_MIN_RAD;
        double max = TurretConstants.PHYSICAL_MAX_RAD;
        double distanceToMin = Math.abs(AngleUtil.wrapRadians(idealWrapped - min));
        double distanceToMax = Math.abs(AngleUtil.wrapRadians(idealWrapped - max));
        if (distanceToMin <= distanceToMax) {
            return min;
        }
        return max;
    }

    /**
     * Continuous unwind. Zero at the soft start, then proportional to how far
     * the commanded angle sits past it, clamped to a maximum rate.
     * Sign matches the turret angle: positive chassis yaw reduces a positive
     * turret angle because desired turret = wrap(bearing - heading).
     */
    private static double requestedChassisOmega(double desiredTurretAngle) {
        double excess = Math.abs(desiredTurretAngle) - TurretConstants.UNWIND_START_RAD;
        if (excess <= 0.0) {
            return 0.0;
        }
        double magnitude = AngleUtil.clamp(
                excess * TurretConstants.UNWIND_KP,
                0.0,
                TurretConstants.MAX_REQUESTED_CHASSIS_OMEGA);
        return Math.copySign(magnitude, desiredTurretAngle);
    }

    private static double blockPowerIntoStop(double turretAngle, double power) {
        if (turretAngle >= TurretConstants.PHYSICAL_MAX_RAD && power > 0.0) {
            return 0.0;
        }
        if (turretAngle <= TurretConstants.PHYSICAL_MIN_RAD && power < 0.0) {
            return 0.0;
        }
        return power;
    }

    private static boolean inputsUsable(TurretState state) {
        return state != null
                && finite(state.robotX)
                && finite(state.robotY)
                && finite(state.robotHeading)
                && finite(state.robotAngularVelocity)
                && finite(state.turretAngle)
                && finite(state.turretVelocity)
                && finite(state.targetX)
                && finite(state.targetY)
                && finite(state.dt);
    }

    private static TurretCommand safeHold(TurretState state) {
        double hold = 0.0;
        if (state != null && finite(state.turretAngle)) {
            hold = AngleUtil.clamp(
                    state.turretAngle,
                    TurretConstants.PHYSICAL_MIN_RAD,
                    TurretConstants.PHYSICAL_MAX_RAD);
        }
        return new TurretCommand(0.0, hold, 0.0, 0.0, 0.0, 0.0, false, false);
    }

    private static boolean finite(double value) {
        return Double.isFinite(value);
    }

    private static final class AngleChoice {
        final double angleRad;
        final boolean reachable;

        AngleChoice(double angleRad, boolean reachable) {
            this.angleRad = angleRad;
            this.reachable = reachable;
        }
    }
}
