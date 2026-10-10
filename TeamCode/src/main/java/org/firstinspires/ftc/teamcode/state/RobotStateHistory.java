package org.firstinspires.ftc.teamcode.state;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Recent {@link RobotState} snapshots for a later timestamp lookup.
 *
 * <p>Call {@link #add(RobotState)} once per control loop from the OpMode
 * thread. There is no background thread. Samples older than
 * {@link #getDurationSec()} drop off the front, and the list also caps at
 * {@link #getMaxSamples()} so a bad clock cannot grow it without limit.
 *
 * <p>{@link #sampleAt(double)} returns the stored snapshot when the request
 * lands on a sample, within {@link #getEndpointToleranceSec()}. That tolerance
 * only absorbs clock alignment. It is not motion extrapolation: a request
 * outside the stored window by more than the tolerance is unavailable.
 * Between two pose-valid samples, position and velocity are linear and
 * heading follows the short way around the circle. An invalid pose in either
 * sample makes the interval unavailable. The result is never a fabricated origin.
 */
public final class RobotStateHistory {

    /** Placeholder window. Long enough for a camera frame, short enough for a match loop. */
    public static final double DEFAULT_DURATION_SEC = 1.5;

    /** About two seconds of a 100 Hz loop, as a hard cap beside the time window. */
    public static final int DEFAULT_MAX_SAMPLES = 200;

    /**
     * Requests this close to a stored sample use that sample.
     * Two milliseconds is alignment slack, not a guess of camera delay.
     */
    public static final double DEFAULT_ENDPOINT_TOLERANCE_SEC = 0.002;

    private final double durationSec;
    private final int maxSamples;
    private final double endpointToleranceSec;
    private final List<RobotState> samples = new ArrayList<RobotState>();

    public RobotStateHistory() {
        this(DEFAULT_DURATION_SEC, DEFAULT_MAX_SAMPLES, DEFAULT_ENDPOINT_TOLERANCE_SEC);
    }

    public RobotStateHistory(double durationSec, int maxSamples, double endpointToleranceSec) {
        this.durationSec = Double.isFinite(durationSec) && durationSec > 0.0 ? durationSec : DEFAULT_DURATION_SEC;
        this.maxSamples = maxSamples > 0 ? maxSamples : DEFAULT_MAX_SAMPLES;
        this.endpointToleranceSec = Double.isFinite(endpointToleranceSec) && endpointToleranceSec >= 0.0
                ? endpointToleranceSec
                : DEFAULT_ENDPOINT_TOLERANCE_SEC;
    }

    public double getDurationSec() {
        return durationSec;
    }

    public int getMaxSamples() {
        return maxSamples;
    }

    public double getEndpointToleranceSec() {
        return endpointToleranceSec;
    }

    public int size() {
        return samples.size();
    }

    public void clear() {
        samples.clear();
    }

    /** Ignores a null state or a non-finite timestamp. Same timestamps replace the older sample. */
    public void add(RobotState state) {
        if (state == null || !Double.isFinite(state.getTimestampSec())) {
            return;
        }
        if(!samples.isEmpty()&&state.getResetGeneration()<samples.get(samples.size()-1).getResetGeneration())return;
        if (!samples.isEmpty() && state.getResetGeneration() != samples.get(samples.size() - 1).getResetGeneration()) {
            samples.clear();
        }
        int index = 0;
        while (index < samples.size()
                && samples.get(index).getTimestampSec() < state.getTimestampSec() - 1.0e-9) {
            index++;
        }
        if (index < samples.size()
                && Math.abs(samples.get(index).getTimestampSec() - state.getTimestampSec()) <= 1.0e-9) {
            samples.set(index, state);
        } else {
            samples.add(index, state);
        }
        double newest = samples.get(samples.size() - 1).getTimestampSec();
        while (!samples.isEmpty() && newest - samples.get(0).getTimestampSec() > durationSec) {
            samples.remove(0);
        }
        while (samples.size() > maxSamples) {
            samples.remove(0);
        }
    }

    /**
     * @return the pose at {@code timestampSec}, or empty when it cannot be reconstructed
     */
    public Optional<RobotState> sampleAt(double timestampSec) {
        if (!Double.isFinite(timestampSec) || samples.isEmpty()) {
            return Optional.empty();
        }
        RobotState oldest = samples.get(0);
        RobotState newest = samples.get(samples.size() - 1);
        double edge = endpointToleranceSec + 1.0e-9;
        if (timestampSec < oldest.getTimestampSec() - edge
                || timestampSec > newest.getTimestampSec() + edge) {
            return Optional.empty();
        }
        int nearest = nearestSample(timestampSec);
        if (nearest >= 0) {
            RobotState exact = samples.get(nearest);
            if (!exact.isPoseValid()) {
                return Optional.empty();
            }
            return Optional.of(exact);
        }
        int upper = 0;
        while (upper < samples.size() && samples.get(upper).getTimestampSec() < timestampSec) {
            upper++;
        }
        if (upper <= 0 || upper >= samples.size()) {
            return Optional.empty();
        }
        RobotState left = samples.get(upper - 1);
        RobotState right = samples.get(upper);
        if (!left.isPoseValid() || !right.isPoseValid()
                || left.getResetGeneration() != right.getResetGeneration()) {
            return Optional.empty();
        }
        double span = right.getTimestampSec() - left.getTimestampSec();
        if (!(span > 0.0) || !Double.isFinite(span)) {
            return Optional.empty();
        }
        double alpha = (timestampSec - left.getTimestampSec()) / span;
        return Optional.of(interpolate(left, right, alpha, timestampSec));
    }

    private int nearestSample(double timestampSec) {
        int nearest = -1;
        double nearestDt = Double.POSITIVE_INFINITY;
        for (int i = 0; i < samples.size(); i++) {
            double dt = Math.abs(samples.get(i).getTimestampSec() - timestampSec);
            if (dt <= endpointToleranceSec + 1.0e-9 && dt < nearestDt) {
                nearest = i;
                nearestDt = dt;
            }
        }
        return nearest;
    }

    private static RobotState interpolate(RobotState left, RobotState right, double alpha, double timestampSec) {
        boolean velocity = left.isVelocityValid() && right.isVelocityValid();
        double headingDelta = AngleUtil.wrapRadians(right.getHeadingRad() - left.getHeadingRad());
        double heading = AngleUtil.wrapRadians(left.getHeadingRad() + (alpha * headingDelta));
        return new RobotState(
                timestampSec,
                lerp(left.getFieldX(), right.getFieldX(), alpha),
                lerp(left.getFieldY(), right.getFieldY(), alpha),
                heading,
                velocity ? lerp(left.getFieldVx(), right.getFieldVx(), alpha) : 0.0,
                velocity ? lerp(left.getFieldVy(), right.getFieldVy(), alpha) : 0.0,
                velocity ? lerp(left.getAngularVelocityRadPerSec(), right.getAngularVelocityRadPerSec(), alpha) : 0.0,
                velocity ? lerp(left.getFieldAx(), right.getFieldAx(), alpha) : 0.0,
                velocity ? lerp(left.getFieldAy(), right.getFieldAy(), alpha) : 0.0,
                velocity ? lerp(left.getAngularAccelerationRadPerSec2(), right.getAngularAccelerationRadPerSec2(), alpha) : 0.0,
                true,
                velocity,
                Math.min(left.getAcquisitionTimestampSec(), right.getAcquisitionTimestampSec()),
                left.isDeviceHealthy() && right.isDeviceHealthy(), left.getResetGeneration());
    }

    private static double lerp(double start, double end, double alpha) {
        return start + (alpha * (end - start));
    }
}
