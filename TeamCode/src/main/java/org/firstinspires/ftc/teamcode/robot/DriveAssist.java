package org.firstinspires.ftc.teamcode.robot;

/** Measured rad/s to normalized turn-power conversion; driver input has priority. */
public final class DriveAssist {
    private final double yawRateAtFullPower,maxAssist,deadband;
    public DriveAssist(double yawRateAtFullPower,double maxAssist,double deadband) {
        if (!Double.isFinite(yawRateAtFullPower)||yawRateAtFullPower<=0||!Double.isFinite(maxAssist)
                ||maxAssist<0||maxAssist>1||!Double.isFinite(deadband)||deadband<0||deadband>=1) throw new IllegalArgumentException("Measured yaw authority required");
        this.yawRateAtFullPower=yawRateAtFullPower;this.maxAssist=maxAssist;this.deadband=deadband;
    }
    public double turn(double driverPower,double requestedRadPerSec,boolean assistEnabled,boolean healthy) {
        if (!healthy||!Double.isFinite(driverPower)||!Double.isFinite(requestedRadPerSec)) return 0;
        if (Math.abs(driverPower)>deadband||!assistEnabled) return Math.max(-1,Math.min(1,driverPower));
        return Math.max(-maxAssist,Math.min(maxAssist,requestedRadPerSec/yawRateAtFullPower));
    }
}
