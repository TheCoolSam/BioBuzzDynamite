package org.firstinspires.ftc.teamcode.turret;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/**
 * Field-relative turret tracker.
 *
 * <p>Call {@link #calculate(TurretState)} once per loop from the OpMode thread.
 * It does not sleep, start threads, or talk to hardware. Motor power is applied
 * by the caller through {@link TurretIO}. Chassis omega is only a request.
 *
 * <p>Reuse one controller for its motion reference and unwind state. Derivative
 * feedback damps measured motion. Reset after faults or localization changes.
 */
public class TurretController {
    private final TurretControlProfile profile;
    private double referenceAngle=Double.NaN,referenceVelocity;
    public TurretController() { this(TurretControlProfile.simulation()); }
    public TurretController(TurretControlProfile profile) {
        if (profile==null) throw new IllegalArgumentException("Turret profile required");
        this.profile=profile;
    }

    /** Ignore the D term when dt is tiny or after a long hitch. */
    private static final double MIN_DT_FOR_DERIVATIVE = 1.0e-4;
    private static final double MAX_DT_FOR_DERIVATIVE = 0.2;

    private boolean hasLastError = false;
    /** One recovery state owns both branch commitment and unwind hysteresis. */
    private enum RecoverySide { NONE, POSITIVE, NEGATIVE }
    private RecoverySide recoverySide = RecoverySide.NONE;
    private double lastUnwindMagnitude;
    private long generation = Long.MIN_VALUE;

    public void reset() {
        referenceAngle=Double.NaN;referenceVelocity=0;
        hasLastError = false;
        recoverySide = RecoverySide.NONE;
        lastUnwindMagnitude = 0.0;
    }

    public TurretCommand calculate(TurretState state) {
        if (state != null && state.resetGeneration != generation) {
            reset(); generation = state.resetGeneration;
        }
        if (state != null && !state.poseValid) {
            reset();
            return holdPosition(state);
        }
        if (!inputsUsable(state)) {
            reset();
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
                profile.OPERATING_MIN_RAD,
                profile.OPERATING_MAX_RAD);
        desired = AngleUtil.clamp(
                desired,
                profile.PHYSICAL_MIN_RAD,
                profile.PHYSICAL_MAX_RAD);

        if (profile.measured) {
            if (!Double.isFinite(referenceAngle)) referenceAngle=AngleUtil.clamp(state.turretAngle,
                    profile.OPERATING_MIN_RAD,profile.OPERATING_MAX_RAD);
            double gap=desired-referenceAngle;
            double step=profile.maxAcceleration*state.dt;
            double velocity=Math.signum(gap)*Math.min(profile.maxSpeed,
                    Math.sqrt(step*step+2*profile.maxAcceleration*Math.abs(gap))-step);
            referenceVelocity+=AngleUtil.clamp(velocity-referenceVelocity,
                    -profile.maxAcceleration*state.dt,profile.maxAcceleration*state.dt);
            double next=referenceAngle+referenceVelocity*state.dt;
            referenceAngle=AngleUtil.clamp(next,profile.OPERATING_MIN_RAD,profile.OPERATING_MAX_RAD);
            if(referenceAngle!=next)referenceVelocity=0; // Final interval guard on abrupt target changes.
            desired=referenceAngle;
        }

        // Do not wrap this error. The short wrap can point through the rear deadzone.
        // The commanded angle stays in the operating window, so the raw difference
        // moves the turret toward that window instead of across the stop.
        double error = desired - state.turretAngle;

        double errorRate = 0.0;
        if (hasLastError
                && state.dt > MIN_DT_FOR_DERIVATIVE
                && state.dt <= MAX_DT_FOR_DERIVATIVE) {
            // D damps measured motion, avoiding target-switch derivative kicks.
            errorRate = -state.turretVelocity;
        }
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
        if (!choice.reachable || desired != choice.angleRad
                || (desired >= profile.OPERATING_MAX_RAD && desiredVelocity > 0)
                || (desired <= profile.OPERATING_MIN_RAD && desiredVelocity < 0)) desiredVelocity = 0;
        if (profile.measured) desiredVelocity=referenceVelocity;
        if (profile.measured) errorRate=desiredVelocity-state.turretVelocity;

        double power = (profile.KP * error)
                + (profile.KD * errorRate)
                + (profile.KV * desiredVelocity);
        if (!Double.isFinite(power)) {
            power = 0.0;
        }
        power = AngleUtil.clamp(
                power,
                profile.MIN_MOTOR_POWER,
                profile.MAX_MOTOR_POWER);
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
                profile.OPERATING_MIN_RAD,
                profile.OPERATING_MAX_RAD);
        if (recoverySide == RecoverySide.NONE) {
            if (match.reachable && Math.abs(match.angleRad) < profile.UNWIND_ENTER_RAD) {
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
        boolean interior = match.reachable && Math.abs(match.angleRad) <= profile.UNWIND_EXIT_RAD;
        if (interior && Math.abs(currentAngle) <= profile.UNWIND_EXIT_RAD) {
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
        return new AngleChoice(positive ? profile.OPERATING_MAX_RAD
                : profile.OPERATING_MIN_RAD, false);
    }

    /** Prefer measured encoder travel; circular target distance only breaks a tie. */
    private double nearestOperatingBoundary(double idealWrapped, double currentAngle) {
        double min = profile.OPERATING_MIN_RAD;
        double max = profile.OPERATING_MAX_RAD;
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
        double interiorTarget = Math.max(0.0, profile.UNWIND_EXIT_RAD
                - 0.5 * (profile.UNWIND_ENTER_RAD - profile.UNWIND_EXIT_RAD));
        double excess = sign * recoveryAngle - interiorTarget;
        double magnitude = AngleUtil.clamp(
                excess * profile.UNWIND_KP,
                0.0,
                profile.MAX_REQUESTED_CHASSIS_OMEGA);
        double rampDt = AngleUtil.clamp(dt, 0.0, MAX_DT_FOR_DERIVATIVE);
        magnitude = Math.min(magnitude, lastUnwindMagnitude
                + profile.UNWIND_OMEGA_RAMP_RAD_PER_SEC2 * rampDt);
        lastUnwindMagnitude = magnitude;
        return sign * magnitude;
    }

    private double blockPowerIntoStop(double turretAngle, double power) {
        if (turretAngle >= profile.PHYSICAL_MAX_RAD && power > 0.0) {
            return 0.0;
        }
        if (turretAngle <= profile.PHYSICAL_MIN_RAD && power < 0.0) {
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
                || !finite(state.dt)
                || state.dt < MIN_DT_FOR_DERIVATIVE
                || state.dt > MAX_DT_FOR_DERIVATIVE) {
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
     * The caller resets recovery and derivative memory before holding.
     */
    private static TurretCommand holdPosition(TurretState state) {
        double hold = 0.0;
        if (state != null && finite(state.turretAngle)) {
            hold = state.turretAngle;
        }
        return new TurretCommand(0.0, hold, 0.0, 0.0, 0.0, 0.0, false, false, false);
    }

    private TurretCommand safeHold(TurretState state) {
        double hold = 0.0;
        if (state != null && finite(state.turretAngle)) {
            hold = AngleUtil.clamp(
                    state.turretAngle,
                    profile.OPERATING_MIN_RAD,
                    profile.OPERATING_MAX_RAD);
            hold = AngleUtil.clamp(
                    hold,
                    profile.PHYSICAL_MIN_RAD,
                    profile.PHYSICAL_MAX_RAD);
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
