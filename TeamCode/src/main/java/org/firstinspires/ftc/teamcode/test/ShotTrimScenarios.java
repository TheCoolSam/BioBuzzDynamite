package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.match.MatchSetup;
import org.firstinspires.ftc.teamcode.scoring.ShotTrim;

/** Desktop checks for driver-2 shot trims and the INIT alliance picker. */
public final class ShotTrimScenarios {
    private static int passed;

    private static void check(boolean ok, String what) {
        if (!ok) throw new AssertionError("FAILED: " + what);
        passed++;
    }

    private static void near(double a, double b, String what) {
        check(Math.abs(a - b) < 1e-9, what + " (" + a + " vs " + b + ")");
    }

    public static void main(String[] args) {
        ShotTrim t = new ShotTrim();
        near(t.rpmScale(40), 1.0, "no trim by default");
        near(t.aimRad(), 0, "no aim trim by default");

        t.shotShort(40);
        near(t.rpmScale(40), 1.0 + ShotTrim.RPM_STEP, "short near shot speeds NEAR band up");
        near(t.rpmScale(80), 1.0, "MID band untouched by a NEAR trim");
        near(t.rpmScale(130), 1.0, "FAR band untouched by a NEAR trim");
        check(t.takeDirty(), "trim change marks the file for saving");
        check(!t.takeDirty(), "dirty flag clears once taken");

        t.shotLong(130);
        t.shotLong(130);
        near(t.rpmScale(130), 1.0 - 2 * ShotTrim.RPM_STEP, "two long far shots slow FAR band twice");

        for (int i = 0; i < 50; i++) t.shotShort(80);
        near(t.rpmScale(80), 1.0 + ShotTrim.RPM_LIMIT, "speed trim is clamped");

        t.shotRight();
        near(t.aimRad(), ShotTrim.AIM_STEP_RAD, "landed right: aim turns left (positive)");
        t.shotLeft();
        t.shotLeft();
        near(t.aimRad(), -ShotTrim.AIM_STEP_RAD, "landed left twice: aim turns right");
        for (int i = 0; i < 50; i++) t.shotRight();
        near(t.aimRad(), ShotTrim.AIM_LIMIT_RAD, "aim trim is clamped");

        near(ShotTrim.band(Double.NaN), 0, "no range counts as NEAR");
        t.made();
        t.made();
        t.missed();
        check(t.madeCount() == 2 && t.missedCount() == 1, "made/missed counted");
        check(t.describe().contains("made 2/3"), "hit rate shown: " + t.describe());

        t.reset();
        near(t.rpmScale(80), 1.0, "reset clears speed trims");
        near(t.aimRad(), 0, "reset clears aim trim");

        t.restore(0.5, Double.NaN, -0.02, Math.toRadians(30));
        near(t.rpmFraction(0), ShotTrim.RPM_LIMIT, "restored values are clamped");
        near(t.rpmFraction(1), 0, "a bad stored value restores as zero");
        near(t.aimRad(), ShotTrim.AIM_LIMIT_RAD, "restored aim clamped");

        check(!MatchSetup.pickFromGamepad(true, true), "both buttons at once pick nothing");
        check(MatchSetup.pickFromGamepad(true, false) && MatchSetup.alliance() == AllianceColor.BLUE, "X picks BLUE");
        check(!MatchSetup.pickFromGamepad(true, false), "holding X does not re-fire");
        check(MatchSetup.pickFromGamepad(false, true) && MatchSetup.alliance() == AllianceColor.RED, "B picks RED");
        MatchSetup.setAlliance(AllianceColor.UNKNOWN);
        check(MatchSetup.alliance() == AllianceColor.RED, "UNKNOWN never overwrites a pick");

        System.out.println("SHOT TRIM CHECKS PASSED (" + passed + ")");
    }
}
