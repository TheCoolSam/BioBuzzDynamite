package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.state.RobotStateSource;

/**
 * One Pedro pose and velocity, already checked, in the project frame.
 *
 * <p>Pedro 3.0.1 uses the same axis directions as
 * {@link org.firstinspires.ftc.teamcode.math.AngleUtil}: +X right, +Y forward,
 * heading 0 along +X, counterclockwise positive. {@link Pose#x()}, {@link Pose#y()},
 * and {@link Pose#heading()} are inches and radians. {@link Velocity#vx},
 * {@link Velocity#vy}, and {@link Velocity#omega} are the world-frame rates
 * on those same axes, in inches per second and radians per second.
 * The FTC-center conversion in Pedro's docs (shift by 72 in and turn by 90°)
 * is not applied. This project never treated the field center as the origin.
 *
 * <p>A null or non-finite pose is not reported as a valid (0, 0, 0). The
 * numbers are zeros so callers never see NaN, and {@link #isPoseValid()} is
 * false. Velocity is independent: a bad velocity does not invalidate a good pose.
 */
public final class PedroMappedReading implements RobotStateSource {

    private final double x;
    private final double y;
    private final double headingRad;
    private final double fieldVx;
    private final double fieldVy;
    private final double omegaRadPerSec;
    private final boolean poseValid;
    private final boolean velocityValid;

    private PedroMappedReading(
            double x,
            double y,
            double headingRad,
            double fieldVx,
            double fieldVy,
            double omegaRadPerSec,
            boolean poseValid,
            boolean velocityValid) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
        this.fieldVx = fieldVx;
        this.fieldVy = fieldVy;
        this.omegaRadPerSec = omegaRadPerSec;
        this.poseValid = poseValid;
        this.velocityValid = velocityValid;
    }

    public static PedroMappedReading from(Pose pose, Velocity velocity) {
        boolean poseValid = pose != null
                && finite(pose.x())
                && finite(pose.y())
                && finite(pose.heading());
        boolean velocityValid = velocity != null
                && finite(velocity.vx)
                && finite(velocity.vy)
                && finite(velocity.omega);
        return new PedroMappedReading(
                poseValid ? pose.x() : 0.0,
                poseValid ? pose.y() : 0.0,
                poseValid ? pose.heading() : 0.0,
                velocityValid ? velocity.vx : 0.0,
                velocityValid ? velocity.vy : 0.0,
                velocityValid ? velocity.omega : 0.0,
                poseValid,
                velocityValid);
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getHeadingRad() {
        return headingRad;
    }

    @Override
    public double getFieldVx() {
        return fieldVx;
    }

    @Override
    public double getFieldVy() {
        return fieldVy;
    }

    @Override
    public double getAngularVelocityRadPerSec() {
        return omegaRadPerSec;
    }

    @Override
    public boolean isPoseValid() {
        return poseValid;
    }

    @Override
    public boolean isVelocityValid() {
        return velocityValid;
    }

    private static boolean finite(double value) {
        return Double.isFinite(value);
    }
}
