package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.scoring.FlywheelAutoTuner;

import java.util.function.DoubleUnaryOperator;

/**
 * Runs the PIDF auto-tuner against a simulated flywheel: first-order speed response with a
 * friction floor, driven by the same battery-compensated software loop FlywheelPair uses.
 */
public final class FlywheelAutoTunerScenarios {
    private static int passed;

    private static void check(boolean ok, String what) {
        if (!ok) throw new AssertionError("FAILED: " + what);
        passed++;
    }

    /** Simulated wheel. Speeds in ticks per second; 28 ticks per motor rev. */
    private static final class Wheel {
        final double topTps, kS, tau, volts;
        double tps, integral, previousI;
        Wheel(double topTps, double kS, double tau, double volts) {
            this.topTps = topTps; this.kS = kS; this.tau = tau; this.volts = volts;
        }
        void step(FlywheelAutoTuner.Output o, double dt) {
            double u;
            if (!o.on) { u = 0; integral = 0; }
            else if (!o.velocityMode) { u = o.power; integral = 0; }
            else {
                if (o.i != previousI) integral = 0;
                previousI = o.i;
                double target = o.targetRpm * 28 / 60.0, err = target - tps, scale = 12.0 / volts;
                if (Math.abs(err) < 0.15 * target) integral = Math.max(-0.15, Math.min(0.15, integral + o.i * err * dt / 32767.0));
                u = Math.max(0, Math.min(1, (o.f * target + o.p * err) * scale / 32767.0 + integral));
            }
            double uEff = u * volts / 12.0;
            double steady = uEff <= kS ? 0 : topTps * (uEff - kS) / (1 - kS);
            tps += (steady - tps) * dt / tau;
        }
        double rpm() { return tps * 60 / 28.0; }
    }

    private static FlywheelAutoTuner run(Wheel w, double targetRpm) {
        return run(w, targetRpm, DoubleUnaryOperator.identity());
    }

    private static FlywheelAutoTuner run(Wheel w, double targetRpm, DoubleUnaryOperator holdReading) {
        FlywheelAutoTuner t = new FlywheelAutoTuner(targetRpm, 28, 6500);
        double dt = 0.02, now = 0;
        double filteredRpm = 0;
        t.start(now);
        while (t.running() && now < 180) {
            double rpm = t.phase() == FlywheelAutoTuner.Phase.I_CHECK ? holdReading.applyAsDouble(w.rpm()) : w.rpm();
            filteredRpm += 0.35 * (rpm - filteredRpm);
            FlywheelAutoTuner.Output o = t.update(now, filteredRpm, w.volts);
            w.step(o, dt);
            now += dt;
        }
        return t;
    }

    public static void main(String[] args) {
        // Typical 6000 rpm motor with a flywheel, fresh battery.
        Wheel w = new Wheel(2700, 0.04, 0.35, 12.8);
        FlywheelAutoTuner t = run(w, 3500);
        check(t.phase() == FlywheelAutoTuner.Phase.DONE, "tunes a normal flywheel: " + t.message());
        double targetTps = 3500 * 28 / 60.0;
        double trueF = 32767 * (0.04 + (1 - 0.04) / 2700 * targetTps) / targetTps;
        check(Math.abs(t.resultF() - trueF) / trueF < 0.05, "F within 5% of the true value (" + t.resultF() + " vs " + trueF + ")");
        check(t.resultP() > 0 && t.resultI() > 0, "P and I chosen");
        check(t.holdErrorPct() <= 1.0, "holds the target within 1% (" + t.holdErrorPct() + "%)");
        check(Math.abs(t.topRpm() - 2700 * 60 / 28.0) < 300, "top speed estimated (" + t.topRpm() + ")");
        check(t.pReport().size() == 4, "every P candidate scored");

        // Same wheel on a tired battery: F is normalised to 12 V, so the answer barely moves.
        FlywheelAutoTuner low = run(new Wheel(2700, 0.04, 0.35, 11.6), 3500);
        check(low.phase() == FlywheelAutoTuner.Phase.DONE, "tunes on a low battery: " + low.message());
        check(Math.abs(low.resultF() - t.resultF()) / t.resultF() < 0.05, "F does not depend on battery voltage");

        // Target too close to the top speed is refused instead of tuning a motor that will cook.
        FlywheelAutoTuner tooFast = run(new Wheel(2700, 0.04, 0.35, 12.5), 5600);
        check(tooFast.phase() == FlywheelAutoTuner.Phase.FAILED && tooFast.message().contains("top speed"), "refuses a target near top speed: " + tooFast.message());

        FlywheelAutoTuner notEnoughVoltage = run(new Wheel(2700, 0.04, 0.35, 11.0), 5000);
        check(notEnoughVoltage.phase() == FlywheelAutoTuner.Phase.FAILED && notEnoughVoltage.message().contains("at 11.0 V"),
                "checks RPM headroom at the measured voltage: " + notEnoughVoltage.message());

        // Encoder unplugged: speed stays 0, the tuner stops instead of running blind.
        FlywheelAutoTuner dead = run(new Wheel(0.0001, 0.99, 0.35, 12.5), 3500);
        check(dead.phase() == FlywheelAutoTuner.Phase.FAILED && dead.message().contains("not spinning"), "detects a wheel that never spins: " + dead.message());

        FlywheelAutoTuner lostEncoder = run(new Wheel(2700, 0.04, 0.35, 12.5), 3500, rpm -> 0);
        check(lostEncoder.phase() == FlywheelAutoTuner.Phase.FAILED && lostEncoder.message().contains("not spinning"),
                "detects an encoder loss after the sweep: " + lostEncoder.message());

        FlywheelAutoTuner badHold = run(new Wheel(2700, 0.04, 0.35, 12.5), 3500, rpm -> 3300);
        check(badHold.phase() == FlywheelAutoTuner.Phase.FAILED && badHold.message().contains("hold exceeded limits"),
                "rejects gains after all I trials miss the target: " + badHold.message());

        FlywheelAutoTuner slowSweep = new FlywheelAutoTuner(3500, 28, 6500);
        slowSweep.start(0);
        double advancedAt = Double.NaN;
        for (int i = 0; i < 750 && slowSweep.running(); i++) {
            double now = i * 0.02;
            double rpm = Math.min(2300, 200 + 300 * now) + (i % 2 == 0 ? -45 : 45);
            FlywheelAutoTuner.Output output = slowSweep.update(now, rpm, 12);
            if (output.power > 0.45) {
                advancedAt = now;
                break;
            }
        }
        check(advancedAt >= 7 && advancedAt < 15,
                "gives a slow spin-up time to settle and tolerates small encoder jitter: " + slowSweep.message());

        FlywheelAutoTuner unstableSweep = runFirstSweep(now -> 1100 + 100 * Math.sin(20 * now));
        check(unstableSweep.phase() == FlywheelAutoTuner.Phase.FAILED && unstableSweep.message().contains("did not settle"),
                "rejects a noisy sweep even if its mean speed is constant: " + unstableSweep.message());
        check(unstableSweep.sweepElapsedSec() >= 15, "reports how long the timed-out step actually ran");
        FlywheelAutoTuner driftingSweep = runFirstSweep(now -> 1000 + 60 * now);
        check(driftingSweep.phase() == FlywheelAutoTuner.Phase.FAILED && driftingSweep.message().contains("did not settle"),
                "rejects a low-noise sweep while the speed is still climbing: " + driftingSweep.message());

        // Abort works.
        FlywheelAutoTuner a = new FlywheelAutoTuner(3500, 28, 6500);
        a.start(0);
        a.abort("driver");
        check(!a.running() && !a.update(1, 1000, 12).on, "abort turns the flywheel off");

        System.out.println("FLYWHEEL AUTO-TUNE CHECKS PASSED (" + passed + ")");
    }

    private static FlywheelAutoTuner runFirstSweep(DoubleUnaryOperator rpmAtTime) {
        FlywheelAutoTuner tuner = new FlywheelAutoTuner(3500, 28, 6500);
        tuner.start(0);
        for (int i = 0; i <= 800 && tuner.running(); i++) {
            double now = i * 0.02;
            FlywheelAutoTuner.Output output = tuner.update(now, rpmAtTime.applyAsDouble(now), 12);
            if (output.power > 0.45) throw new AssertionError("Unstable first sweep advanced");
        }
        return tuner;
    }
}
