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
    private PedroMappedReading reading = PedroMappedReading.from(null, null);
    private double acquiredSec = Double.NaN;
    private boolean healthy;
    private long generation;

    /** Call only after a successful localizer acquisition and verified device health. */
    public void capture(double acquisitionSec, boolean deviceHealthy) {
        healthy = deviceHealthy && Double.isFinite(acquisitionSec)
                && (!Double.isFinite(acquiredSec) || acquisitionSec >= acquiredSec);
        reading = PedroMappedReading.from(follower.pose(), follower.velocity(),acquisitionSec,healthy,generation);
        if (healthy) acquiredSec = acquisitionSec;
    }

    public void invalidate() { healthy = false; }
    public void localizationReset() {
        generation++; acquiredSec = Double.NaN; healthy = false;
        reading = PedroMappedReading.from(null, null);
    }
    @Override public double getAcquisitionTimestampSec() { return acquiredSec; }
    @Override public boolean isDeviceHealthy() { return healthy; }
    @Override public long getResetGeneration() { return generation; }

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
        return reading;
    }
}
