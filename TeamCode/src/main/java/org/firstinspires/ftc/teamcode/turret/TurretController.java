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
    private boolean unwindLatched = false;

    public void reset() {
        lastErrorRad = 0.0;
        hasLastError = false;
        unwindLatched = false;
    }

    public TurretCommand calculate(TurretState state) {
        if (state != null && !state.poseValid) {
            return holdPosition(state);
        }
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
        // Command inside the operating window. Physical stops are only a backstop.
        double desired = AngleUtil.clamp(
                choice.angleRad,
                TurretConstants.OPERATING_MIN_RAD,
                TurretConstants.OPERATING_MAX_RAD);
        desired = AngleUtil.clamp(
                desired,
                TurretConstants.PHYSICAL_MIN_RAD,
                TurretConstants.PHYSICAL_MAX_RAD);

        // Do not wrap this error. The short wrap can point through the rear deadzone.
        // The commanded angle stays in the operating window, so the raw difference
        // moves the turret toward that window instead of across the stop.
        double error = desired - state.turretAngle;

        double errorRate = 0.0;
        if (hasLastError
                && state.dt > MIN_DT_FOR_DERIVATIVE
                && state.dt <= MAX_DT_FOR_DERIVATIVE) {
            errorRate = (error - lastErrorRad) / state.dt;
        }
        lastErrorRad = error;
        hasLastError = true;

        // Turret goal is bearing - heading, so its rate is bearing rate minus chassis yaw.
        // An untrusted velocity contributes no feedforward. Position aiming still runs.
        double desiredVelocity = 0.0;
        if (state.velocityValid) {
            double dx = state.targetX - state.robotX;
            double dy = state.targetY - state.robotY;
            desiredVelocity = AngleUtil.bearingRate(dx, dy, state.robotVx, state.robotVy)
                    - state.robotAngularVelocity;
            if (!Double.isFinite(desiredVelocity)) {
                desiredVelocity = 0.0;
            }
        }

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

        boolean unwindActive = updateUnwindLatch(desired);
        double chassisOmega = 0.0;
        if (unwindActive) {
            chassisOmega = requestedChassisOmega(desired);
        }

        return new TurretCommand(
                bearing,
                desired,
                error,
                desiredVelocity,
                power,
                chassisOmega,
                choice.reachable,
                unwindActive,
                true);
    }

    /**
     * Prefer a 2π-equivalent aim inside the operating window, closest to where
     * the turret already is. If none fits, hold the nearer operating boundary
     * and let chassis unwind bring the target back in. Do not aim at a physical stop.
     */
    private static AngleChoice chooseReachableAngle(double idealWrapped, double currentAngle) {
        AngleUtil.EquivalentAngle match = AngleUtil.closestEquivalentInInterval(
                idealWrapped,
                currentAngle,
                TurretConstants.OPERATING_MIN_RAD,
                TurretConstants.OPERATING_MAX_RAD);
        if (match.reachable) {
            return new AngleChoice(match.angleRad, true);
        }
        return new AngleChoice(nearestOperatingBoundary(idealWrapped), false);
    }

    /** Closer operating boundary, measured the short way around the circle. */
    private static double nearestOperatingBoundary(double idealWrapped) {
        double min = TurretConstants.OPERATING_MIN_RAD;
        double max = TurretConstants.OPERATING_MAX_RAD;
        double distanceToMin = Math.abs(AngleUtil.wrapRadians(idealWrapped - min));
        double distanceToMax = Math.abs(AngleUtil.wrapRadians(idealWrapped - max));
        if (distanceToMin <= distanceToMax) {
            return min;
        }
        return max;
    }

    /**
     * Enter at {@link TurretConstants#UNWIND_ENTER_RAD}, stay on until the command
     * is back inside {@link TurretConstants#UNWIND_EXIT_RAD}. The chassis request
     * itself still grows only past the enter angle, so latching does not step the omega.
     */
    private boolean updateUnwindLatch(double desiredTurretAngle) {
        double magnitude = Math.abs(desiredTurretAngle);
        if (unwindLatched) {
            if (magnitude <= TurretConstants.UNWIND_EXIT_RAD) {
                unwindLatched = false;
            }
        } else if (magnitude >= TurretConstants.UNWIND_ENTER_RAD) {
            unwindLatched = true;
        }
        return unwindLatched;
    }

    /**
     * Continuous unwind. Zero at the enter angle, then proportional to how far
     * the commanded angle sits past it, clamped to a maximum rate.
     * Sign matches the turret angle: positive chassis yaw reduces a positive
     * turret angle because desired turret = wrap(bearing - heading).
     */
    private static double requestedChassisOmega(double desiredTurretAngle) {
        double excess = Math.abs(desiredTurretAngle) - TurretConstants.UNWIND_ENTER_RAD;
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
        if (state == null
                || !finite(state.robotX)
                || !finite(state.robotY)
                || !finite(state.robotHeading)
                || !finite(state.turretAngle)
                || !finite(state.turretVelocity)
                || !finite(state.targetX)
                || !finite(state.targetY)
                || !finite(state.dt)) {
            return false;
        }
        if (!state.velocityValid) {
            return true;
        }
        return finite(state.robotVx)
                && finite(state.robotVy)
                && finite(state.robotAngularVelocity);
    }

    /**
     * Pose cannot be aimed. Keep the measured turret angle, apply no power,
     * and do not ask the chassis to unwind from geometry we do not trust.
     * Does not change the unwind latch or the derivative memory.
     */
    private static TurretCommand holdPosition(TurretState state) {
        double hold = 0.0;
        if (state != null && finite(state.turretAngle)) {
            hold = state.turretAngle;
        }
        return new TurretCommand(0.0, hold, 0.0, 0.0, 0.0, 0.0, false, false, false);
    }

    private static TurretCommand safeHold(TurretState state) {
        double hold = 0.0;
        if (state != null && finite(state.turretAngle)) {
            hold = AngleUtil.clamp(
                    state.turretAngle,
                    TurretConstants.OPERATING_MIN_RAD,
                    TurretConstants.OPERATING_MAX_RAD);
            hold = AngleUtil.clamp(
                    hold,
                    TurretConstants.PHYSICAL_MIN_RAD,
                    TurretConstants.PHYSICAL_MAX_RAD);
        }
        return new TurretCommand(0.0, hold, 0.0, 0.0, 0.0, 0.0, false, false, false);
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
