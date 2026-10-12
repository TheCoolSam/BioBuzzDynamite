package org.firstinspires.ftc.teamcode.turret;

/**
 * Normalized CR demand to a pulse/PWM action. Stop and E-stop write the
 * calibrated neutral (when known) and then disable PWM. A CR servo has no
 * brake: PWM disable can backdrive; mechanical stops and the operator remain
 * required. Uncalibrated CR never writes SDK zero, which may not be stopped.
 */
public final class TurretCrCommand {
    public final double sdkPower;
    public final double pulseUs;
    public final boolean pwmEnabled;
    public final String reason;

    private TurretCrCommand(double sdkPower, double pulseUs, boolean pwmEnabled, String reason) {
        this.sdkPower = sdkPower;
        this.pulseUs = pulseUs;
        this.pwmEnabled = pwmEnabled;
        this.reason = reason == null ? "" : reason;
    }

    public static TurretCrCommand stop(AxonCrCalibration calibration) {
        if (calibration == null) {
            return new TurretCrCommand(Double.NaN, Double.NaN, false, "CR UNCALIBRATED / PWM DISABLED");
        }
        return new TurretCrCommand(calibration.sdkPower(0), calibration.neutralUs, false,
                "CR NEUTRAL THEN PWM DISABLE");
    }

    public static TurretCrCommand drive(AxonCrCalibration calibration, double demand) {
        if (calibration == null || !Double.isFinite(demand) || Math.abs(demand) > 1) {
            return stop(calibration);
        }
        if (demand == 0) return stop(calibration);
        return new TurretCrCommand(calibration.sdkPower(demand), calibration.pulseUs(demand), true, "");
    }

    public static TurretCrCommand measure(double pulseUs, double minUs, double maxUs, double frameUs) {
        if (!Double.isFinite(pulseUs) || !Double.isFinite(minUs) || !Double.isFinite(maxUs)
                || !Double.isFinite(frameUs) || minUs <= 0 || maxUs <= minUs || pulseUs < minUs
                || pulseUs > maxUs || frameUs <= maxUs) {
            return new TurretCrCommand(Double.NaN, Double.NaN, false, "CR MEASURE WINDOW UNCONFIGURED");
        }
        return new TurretCrCommand(Double.NaN, pulseUs, true, "");
    }

    public boolean writesOutput() {
        return pwmEnabled && (Double.isFinite(sdkPower) || Double.isFinite(pulseUs));
    }
}
