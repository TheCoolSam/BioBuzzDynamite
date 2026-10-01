package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.state.RobotStateSource;

/**
 * Reads the Pedro follower into {@link RobotStateSource}.
 *
 * <p>This class does not call {@link Follower#update()}. The OpMode that owns
 * the follower updates it once per loop, then asks the estimator to read this
 * source. Turret, planner, and {@link org.firstinspires.ftc.teamcode.state.RobotStateEstimator}
 * must not update Pedro themselves.
 *
 * <p>Mapping is identity after a finite check. See {@link PedroMappedReading}.
 */
public final class PedroRobotStateSource implements RobotStateSource {

    private final Follower follower;

    public PedroRobotStateSource(Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("follower is required");
        }
        this.follower = follower;
    }

    @Override
    public double getX() {
        return current().getX();
    }

    @Override
    public double getY() {
        return current().getY();
    }

    @Override
    public double getHeadingRad() {
        return current().getHeadingRad();
    }

    @Override
    public double getFieldVx() {
        return current().getFieldVx();
    }

    @Override
    public double getFieldVy() {
        return current().getFieldVy();
    }

    @Override
    public double getAngularVelocityRadPerSec() {
        return current().getAngularVelocityRadPerSec();
    }

    @Override
    public boolean isPoseValid() {
        return current().isPoseValid();
    }

    @Override
    public boolean isVelocityValid() {
        return current().isVelocityValid();
    }

    private PedroMappedReading current() {
        return PedroMappedReading.from(follower.pose(), follower.velocity());
    }
}
