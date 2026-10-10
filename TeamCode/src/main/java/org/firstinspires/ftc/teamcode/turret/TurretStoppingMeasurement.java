package org.firstinspires.ftc.teamcode.turret;

/** Most recent powered -> zero event. Motion uses continuous legal angle, not rear-sector wrap. */
public final class TurretStoppingMeasurement {
    public double zeroAngleRad = Double.NaN, extremeAngleRad = Double.NaN;
    public double zeroTimeSec = Double.NaN, zeroVelocityRadPerSec = Double.NaN;
    public double zeroSampleAgeSec = Double.NaN;
    public double distanceRad = Double.NaN;
    public boolean active, incomplete;
    private double previousPower, direction;
    public boolean update(double power, TurretEncoderSample sample, double nowSec) {
        return update(power, sample, nowSec, 0);
    }
    public boolean update(double power, TurretEncoderSample sample, double nowSec, double sampleAgeSec) {
        boolean event = previousPower != 0 && Double.isFinite(previousPower) && power == 0;
        if (event) {
            active = sample.valid;
            incomplete = !sample.valid;
            zeroAngleRad = extremeAngleRad = sample.valid ? sample.angleRad : Double.NaN;
            zeroVelocityRadPerSec = sample.velocityRadPerSec;
            zeroTimeSec = nowSec;
            zeroSampleAgeSec = sampleAgeSec;
            direction = sample.valid && Math.abs(sample.velocityRadPerSec) > 1e-6
                    ? Math.signum(sample.velocityRadPerSec) : Math.signum(previousPower);
            distanceRad = active ? 0 : Double.NaN;
        }
        if (active && power == 0) {
            if (!sample.valid) { active = false; incomplete = true; }
            else if (direction * (sample.angleRad - extremeAngleRad) > 0) {
                extremeAngleRad = sample.angleRad;
                distanceRad = direction * (extremeAngleRad - zeroAngleRad);
            }
        }
        if (power != 0) active = false;
        previousPower = power;
        return event;
    }
}
