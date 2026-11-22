package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.scoring.FlywheelAutoTuner;

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
        double tps, integral;
        Wheel(double topTps, double kS, double tau, double volts) {
            this.topTps = topTps; this.kS = kS; this.tau = tau; this.volts = volts;
        }
        void step(FlywheelAutoTuner.Output o, double dt) {
            double u;
            if (!o.on) { u = 0; integral = 0; }
            else if (!o.velocityMode) { u = o.power; integral = 0; }
            else {
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
        FlywheelAutoTuner t = new FlywheelAutoTuner(targetRpm, 28, 6500);
        double dt = 0.02, now = 0;
        t.start(now);
        while (t.running() && now < 180) {
            FlywheelAutoTuner.Output o = t.update(now, w.rpm(), w.volts);
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

        // Encoder unplugged: speed stays 0, the tuner stops instead of running blind.
        FlywheelAutoTuner dead = run(new Wheel(0.0001, 0.99, 0.35, 12.5), 3500);
        check(dead.phase() == FlywheelAutoTuner.Phase.FAILED && dead.message().contains("not spinning"), "detects a wheel that never spins: " + dead.message());

        // Abort works.
        FlywheelAutoTuner a = new FlywheelAutoTuner(3500, 28, 6500);
        a.start(0);
        a.abort("driver");
        check(!a.running() && !a.update(1, 1000, 12).on, "abort turns the flywheel off");

        System.out.println("FLYWHEEL AUTO-TUNE CHECKS PASSED (" + passed + ")");
    }
}
