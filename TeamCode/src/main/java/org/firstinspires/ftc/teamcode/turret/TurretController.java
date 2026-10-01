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
    /** One recovery state owns both branch commitment and unwind hysteresis. */
    private enum RecoverySide { NONE, POSITIVE, NEGATIVE }
    private RecoverySide recoverySide = RecoverySide.NONE;
    private double lastUnwindMagnitude;

    public void reset() {
        lastErrorRad = 0.0;
        hasLastError = false;
        recoverySide = RecoverySide.NONE;
        lastUnwindMagnitude = 0.0;
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

        boolean unwindActive = recoverySide != RecoverySide.NONE;
        double chassisOmega = 0.0;
        if (unwindActive) {
            chassisOmega = requestedChassisOmega(ideal, choice, state.dt);
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
     * the turret already is. Recovery commits to a side until both the target
     * geometry and measured turret are inside the exit region. Raw angle moves
     * between ordinary legal targets stay in the operating interval (via front).
     */
    private AngleChoice chooseReachableAngle(double idealWrapped, double currentAngle) {
        AngleUtil.EquivalentAngle match = AngleUtil.closestEquivalentInInterval(
                idealWrapped,
                currentAngle,
                TurretConstants.OPERATING_MIN_RAD,
                TurretConstants.OPERATING_MAX_RAD);
        if (recoverySide == RecoverySide.NONE) {
            if (match.reachable && Math.abs(match.angleRad) < TurretConstants.UNWIND_ENTER_RAD) {
                return new AngleChoice(match.angleRad, true);
            }
            // For an unreachable target choose the legal boundary requiring the
            // least encoder travel through the front, rather than through the rear.
            double selected = match.reachable ? match.angleRad
                    : nearestOperatingBoundary(idealWrapped, currentAngle);
            recoverySide = selected >= 0.0 ? RecoverySide.POSITIVE : RecoverySide.NEGATIVE;
            lastUnwindMagnitude = 0.0;
        }

        boolean positive = recoverySide == RecoverySide.POSITIVE;
        boolean interior = match.reachable && Math.abs(match.angleRad) <= TurretConstants.UNWIND_EXIT_RAD;
        if (interior && Math.abs(currentAngle) <= TurretConstants.UNWIND_EXIT_RAD) {
            recoverySide = RecoverySide.NONE;
            lastUnwindMagnitude = 0.0;
            return new AngleChoice(match.angleRad, true);
        }
        // Resume legal tracking on our side as the chassis recovers. An interior
        // target on either side can also be reached safely via the front; wait
        // for the measured turret to settle before releasing the recovery state.
        if (match.reachable && (interior || (positive ? match.angleRad >= 0 : match.angleRad <= 0))) {
            return new AngleChoice(match.angleRad, true);
        }
        return new AngleChoice(positive ? TurretConstants.OPERATING_MAX_RAD
                : TurretConstants.OPERATING_MIN_RAD, false);
    }

    /** Prefer measured encoder travel; circular target distance only breaks a tie. */
    private static double nearestOperatingBoundary(double idealWrapped, double currentAngle) {
        double min = TurretConstants.OPERATING_MIN_RAD;
        double max = TurretConstants.OPERATING_MAX_RAD;
        double distanceToMin = Math.abs(currentAngle - min);
        double distanceToMax = Math.abs(currentAngle - max);
        if (Math.abs(distanceToMin - distanceToMax) <= 1e-9) {
            distanceToMin = Math.abs(AngleUtil.wrapRadians(idealWrapped - min));
            distanceToMax = Math.abs(AngleUtil.wrapRadians(idealWrapped - max));
        }
        if (distanceToMin <= distanceToMax) {
            return min;
        }
        return max;
    }

    /**
     * Recover toward a point half the hysteresis gap inside EXIT, so finite
     * execution crosses EXIT rather than asymptotically stopping there. Lift an
     * unreachable wrapped bearing onto the committed side of the rear sector;
     * ±pi noise cannot reverse yaw. Ramp increases to avoid an entry power step.
     */
    private double requestedChassisOmega(double idealWrapped, AngleChoice choice, double dt) {
        double sign = recoverySide == RecoverySide.POSITIVE ? 1.0 : -1.0;
        double recoveryAngle = choice.angleRad;
        if (!choice.reachable) {
            recoveryAngle = idealWrapped;
            if (sign * recoveryAngle < 0.0) recoveryAngle += sign * AngleUtil.TAU;
        }
        double interiorTarget = Math.max(0.0, TurretConstants.UNWIND_EXIT_RAD
                - 0.5 * (TurretConstants.UNWIND_ENTER_RAD - TurretConstants.UNWIND_EXIT_RAD));
        double excess = sign * recoveryAngle - interiorTarget;
        double magnitude = AngleUtil.clamp(
                excess * TurretConstants.UNWIND_KP,
                0.0,
                TurretConstants.MAX_REQUESTED_CHASSIS_OMEGA);
        double rampDt = AngleUtil.clamp(dt, 0.0, MAX_DT_FOR_DERIVATIVE);
        magnitude = Math.min(magnitude, lastUnwindMagnitude
                + TurretConstants.UNWIND_OMEGA_RAMP_RAD_PER_SEC2 * rampDt);
        lastUnwindMagnitude = magnitude;
        return sign * magnitude;
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
