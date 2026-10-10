package org.firstinspires.ftc.teamcode.state;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

import java.util.Locale;

/**
 * One immutable estimate of the robot on the field.
 *
 * <p>Distances are inches. Velocities are inches per second. Accelerations are
 * inches per second squared. Angles are radians in the frame documented on
 * {@link AngleUtil}: +X field right, +Y field forward, heading 0 along +X,
 * positive rotation counterclockwise.
 *
 * <p>Check {@link #isPoseValid()} and {@link #isVelocityValid()} before trusting
 * the numbers. Physical pose/rate values are sanitized; unavailable acquisition
 * provenance may be NaN and fails freshness checks. Nothing in here can be changed
 * after it is created, so one subsystem cannot overwrite a value another
 * subsystem is still reading.
 */
public final class RobotState {

    private final double timestampSec;
    private final double fieldX;
    private final double fieldY;
    private final double headingRad;
    private final double fieldVx;
    private final double fieldVy;
    private final double angularVelocityRadPerSec;
    private final double fieldAx;
    private final double fieldAy;
    private final double angularAccelerationRadPerSec2;
    private final boolean poseValid;
    private final boolean velocityValid;
    private final double acquisitionTimestampSec;
    private final boolean deviceHealthy;
    private final long resetGeneration;

    public RobotState(
            double timestampSec,
            double fieldX,
            double fieldY,
            double headingRad,
            double fieldVx,
            double fieldVy,
            double angularVelocityRadPerSec,
            double fieldAx,
            double fieldAy,
            double angularAccelerationRadPerSec2,
            boolean poseValid,
            boolean velocityValid) {
        this(timestampSec, fieldX, fieldY, headingRad, fieldVx, fieldVy,
                angularVelocityRadPerSec, fieldAx, fieldAy, angularAccelerationRadPerSec2,
                poseValid, velocityValid, timestampSec, true, 0);
    }

    public RobotState(double timestampSec, double fieldX, double fieldY, double headingRad,
            double fieldVx, double fieldVy, double angularVelocityRadPerSec,
            double fieldAx, double fieldAy, double angularAccelerationRadPerSec2,
            boolean poseValid, boolean velocityValid, double acquisitionTimestampSec,
            boolean deviceHealthy, long resetGeneration) {
        this.timestampSec = finiteOrZero(timestampSec);
        this.acquisitionTimestampSec = acquisitionTimestampSec;
        this.deviceHealthy = deviceHealthy && finite(timestampSec) && finite(acquisitionTimestampSec);
        this.resetGeneration = resetGeneration;

        if (finite(fieldX) && finite(fieldY) && finite(headingRad)) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.headingRad = AngleUtil.wrapRadians(headingRad);
            this.poseValid = poseValid;
        } else {
            this.fieldX = 0.0;
            this.fieldY = 0.0;
            this.headingRad = 0.0;
            this.poseValid = false;
        }

        if (finite(fieldVx) && finite(fieldVy) && finite(angularVelocityRadPerSec)) {
            this.fieldVx = fieldVx;
            this.fieldVy = fieldVy;
            this.angularVelocityRadPerSec = angularVelocityRadPerSec;
            this.velocityValid = velocityValid;
        } else {
            this.fieldVx = 0.0;
            this.fieldVy = 0.0;
            this.angularVelocityRadPerSec = 0.0;
            this.velocityValid = false;
        }

        if (finite(fieldAx) && finite(fieldAy) && finite(angularAccelerationRadPerSec2)) {
            this.fieldAx = fieldAx;
            this.fieldAy = fieldAy;
            this.angularAccelerationRadPerSec2 = angularAccelerationRadPerSec2;
        } else {
            this.fieldAx = 0.0;
            this.fieldAy = 0.0;
            this.angularAccelerationRadPerSec2 = 0.0;
        }
    }

    /** Snapshot whose numbers are zero and both validity flags are false. */
    public static RobotState invalid(double timestampSec) {
        return new RobotState(
                timestampSec,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                false,
                false);
    }

    public double getTimestampSec() {
        return timestampSec;
    }

    /** Inches. */
    public double getFieldX() {
        return fieldX;
    }

    /** Inches. */
    public double getFieldY() {
        return fieldY;
    }

    public double getHeadingRad() {
        return headingRad;
    }

    /** Inches per second. */
    public double getFieldVx() {
        return fieldVx;
    }

    /** Inches per second. */
    public double getFieldVy() {
        return fieldVy;
    }

    public double getAngularVelocityRadPerSec() {
        return angularVelocityRadPerSec;
    }

    /** Inches per second squared. */
    public double getFieldAx() {
        return fieldAx;
    }

    /** Inches per second squared. */
    public double getFieldAy() {
        return fieldAy;
    }

    public double getAngularAccelerationRadPerSec2() {
        return angularAccelerationRadPerSec2;
    }

    public boolean isPoseValid() {
        return poseValid && deviceHealthy;
    }

    public boolean isVelocityValid() {
        return velocityValid && deviceHealthy;
    }

    public double getAcquisitionTimestampSec() { return acquisitionTimestampSec; }
    public boolean isDeviceHealthy() { return deviceHealthy; }
    public long getResetGeneration() { return resetGeneration; }

    public boolean isFresh(double nowSec, double maxAgeSec) {
        double age = nowSec - acquisitionTimestampSec;
        double snapshotAge = nowSec - timestampSec;
        return isPoseValid() && finite(nowSec) && finite(maxAgeSec) && maxAgeSec >= 0
                && age >= -1e-9 && age <= maxAgeSec + 1e-9
                && snapshotAge >= -1e-9 && snapshotAge <= maxAgeSec + 1e-9;
    }

    @Override
    public String toString() {
        return String.format(
                Locale.US,
                "t=%.3f pose(%.1f in, %.1f in, %.1f deg) %s vel(%.1f, %.1f in/s, %.2f rad/s) %s",
                timestampSec,
                fieldX,
                fieldY,
                AngleUtil.toDegrees(headingRad),
                poseValid ? "pose-ok" : "pose-INVALID",
                fieldVx,
                fieldVy,
                angularVelocityRadPerSec,
                velocityValid ? "vel-ok" : "vel-INVALID");
    }

    private static boolean finite(double value) {
        return Double.isFinite(value);
    }

    private static double finiteOrZero(double value) {
        return finite(value) ? value : 0.0;
    }
}
