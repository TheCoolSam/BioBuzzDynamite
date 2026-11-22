package org.firstinspires.ftc.teamcode.scoring;

import java.util.Locale;

/**
 * Driver 2's live corrections to the calibrated shot.
 *
 * <p>After each shot, driver 2 says what happened: LONG or SHORT nudges the flywheel speed for the
 * range band the robot is shooting from; LEFT or RIGHT nudges the turret aim for every shot. MADE
 * and MISS only count, so the hit rate is visible. The trims are small and bounded, are saved
 * between matches, and are also written to the shot log so the shot table can be re-fitted later.
 *
 * <p>Bands: NEAR below {@link #NEAR_MAX_IN}, MID up to {@link #MID_MAX_IN}, FAR beyond. A shot
 * that is consistently long only from far away is fixed far away, without touching near shots.
 */
public final class ShotTrim {
    public static final double NEAR_MAX_IN = 60.0;
    public static final double MID_MAX_IN = 100.0;
    /** One press changes the flywheel speed by this fraction (1.5 %). */
    public static final double RPM_STEP = 0.015;
    /** Trims never exceed +/- this fraction of the calibrated speed (10 %). */
    public static final double RPM_LIMIT = 0.10;
    /** One press turns the aim by this much (0.5 degrees). */
    public static final double AIM_STEP_RAD = Math.toRadians(0.5);
    /** Aim trim never exceeds +/- 5 degrees. */
    public static final double AIM_LIMIT_RAD = Math.toRadians(5.0);

    public static final String[] BAND_NAMES = {"NEAR", "MID", "FAR"};

    private final double[] rpmFraction = new double[3];
    private double aimRad;
    private int made, missed;
    private boolean dirty;

    public static int band(double rangeInches) {
        if (!Double.isFinite(rangeInches) || rangeInches < NEAR_MAX_IN) return 0;
        return rangeInches < MID_MAX_IN ? 1 : 2;
    }

    /** Multiplier for the calibrated flywheel speed at this range (1.0 = no trim). */
    public synchronized double rpmScale(double rangeInches) {
        return 1.0 + rpmFraction[band(rangeInches)];
    }

    /** Aim offset added to the calibrated bearing; positive turns the shot left (counter-clockwise). */
    public synchronized double aimRad() { return aimRad; }

    /** The last shot went over or too far: slow the flywheel for this band. */
    public synchronized void shotLong(double rangeInches) { nudgeRpm(band(rangeInches), -1); }

    /** The last shot fell short: speed the flywheel up for this band. */
    public synchronized void shotShort(double rangeInches) { nudgeRpm(band(rangeInches), +1); }

    /** The last shot landed right of the target: aim further left. */
    public synchronized void shotRight() { nudgeAim(+1); }

    /** The last shot landed left of the target: aim further right. */
    public synchronized void shotLeft() { nudgeAim(-1); }

    public synchronized void made() { made++; }

    public synchronized void missed() { missed++; }

    public synchronized void reset() {
        for (int i = 0; i < 3; i++) rpmFraction[i] = 0;
        aimRad = 0;
        dirty = true;
    }

    /** Sets stored values (from the saved file); out-of-range values are clamped. */
    public synchronized void restore(double near, double mid, double far, double aim) {
        rpmFraction[0] = clamp(near, RPM_LIMIT);
        rpmFraction[1] = clamp(mid, RPM_LIMIT);
        rpmFraction[2] = clamp(far, RPM_LIMIT);
        aimRad = clamp(aim, AIM_LIMIT_RAD);
    }

    public synchronized double rpmFraction(int band) { return rpmFraction[band]; }

    public synchronized int madeCount() { return made; }

    public synchronized int missedCount() { return missed; }

    /** True once since the last call when a trim changed and should be saved. */
    public synchronized boolean takeDirty() {
        boolean was = dirty;
        dirty = false;
        return was;
    }

    public synchronized String describe() {
        int total = made + missed;
        return String.format(Locale.US, "speed NEAR %+.1f%% MID %+.1f%% FAR %+.1f%% | aim %+.1f deg | made %d/%d%s",
                rpmFraction[0] * 100, rpmFraction[1] * 100, rpmFraction[2] * 100, Math.toDegrees(aimRad),
                made, total, total > 0 ? String.format(Locale.US, " (%.0f%%)", 100.0 * made / total) : "");
    }

    private void nudgeRpm(int band, int sign) {
        rpmFraction[band] = clamp(rpmFraction[band] + sign * RPM_STEP, RPM_LIMIT);
        dirty = true;
    }

    private void nudgeAim(int sign) {
        aimRad = clamp(aimRad + sign * AIM_STEP_RAD, AIM_LIMIT_RAD);
        dirty = true;
    }

    private static double clamp(double v, double limit) {
        if (!Double.isFinite(v)) return 0;
        return Math.max(-limit, Math.min(limit, v));
    }
}
