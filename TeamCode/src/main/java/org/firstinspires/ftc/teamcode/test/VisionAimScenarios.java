package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.turret.VisionAimController;

/** Desktop closed-loop checks for camera-driven turret aiming. */
public final class VisionAimScenarios {
    private static int checks;

    /** Velocity-type turret (like a CR servo): angle rate = gain * power, measurement arrives late and noisy. */
    private static final class Plant {
        final double degPerSecPerPower;
        final double directionSign;
        final int delayFrames;
        final double noiseDeg;
        double turret;
        final double target;
        final double[] history;
        int index;
        long seed = 12345;

        Plant(double turretStart, double target, double gain, double directionSign, int delayFrames, double noiseDeg) {
            this.turret = turretStart;
            this.target = target;
            this.degPerSecPerPower = gain;
            this.directionSign = directionSign;
            this.delayFrames = delayFrames;
            this.noiseDeg = noiseDeg;
            history = new double[delayFrames + 1];
            for (int i = 0; i < history.length; i++) history[i] = target - turretStart;
        }

        double noise() {
            seed = seed * 6364136223846793005L + 1442695040888963407L;
            double u = ((seed >>> 33) / (double) (1L << 31)) - 0.5;
            return u * 2 * noiseDeg;
        }

        /** Returns the delayed, noisy tx the camera would report this frame. */
        double measure() {
            history[index] = target - turret;
            index = (index + 1) % history.length;
            return history[index] + noise();
        }

        void drive(double power, double dt) { turret += directionSign * degPerSecPerPower * power * dt; }
    }

    private static final class RunResult {
        double finalError, maxOvershoot, secondsToLock = Double.NaN, maxAbsDemand;
        boolean locked;
    }

    private static RunResult run(VisionAimController.Config cfg, Plant plant, double seconds, double offset) {
        VisionAimController aim = new VisionAimController(cfg);
        RunResult r = new RunResult();
        double dt = 0.02;
        double startError = plant.target - plant.turret;
        for (double t = 0; t < seconds; t += dt) {
            double tx = plant.measure();
            VisionAimController.Output out = aim.update(true, tx, dt);
            plant.drive(out.demand, dt);
            r.maxAbsDemand = Math.max(r.maxAbsDemand, Math.abs(out.demand));
            double err = (plant.target - plant.turret) - offset;
            if (Math.signum(err) != Math.signum(startError) && err != 0) {
                r.maxOvershoot = Math.max(r.maxOvershoot, Math.abs(err));
            }
            if (out.locked && Double.isNaN(r.secondsToLock)) r.secondsToLock = t;
            r.locked = out.locked;
        }
        r.finalError = (plant.target - plant.turret) - offset;
        return r;
    }

    public static void main(String[] args) {
        VisionAimController.Config cfg = new VisionAimController.Config();

        // 1. Converges from a large offset, locks, bounded overshoot, respects the power cap.
        RunResult r = run(cfg, new Plant(0, 30, 120, +1, 3, 0.2), 8.0, 0);
        check(Math.abs(r.finalError) < 1.0, "converges within 1 degree from 30 degrees (" + r.finalError + ")");
        check(r.locked, "reports lock after convergence");
        check(r.maxOvershoot < 6.0, "overshoot stays small (" + r.maxOvershoot + ")");
        check(r.maxAbsDemand <= cfg.maxPower + 1e-12, "demand never exceeds maxPower");

        // 2. Target on the other side.
        r = run(cfg, new Plant(0, -25, 120, +1, 3, 0.2), 8.0, 0);
        check(Math.abs(r.finalError) < 1.0, "converges from the negative side (" + r.finalError + ")");

        // 3. Flipping sign fixes an inverted mechanism; the wrong sign must run away (so it is detectable).
        VisionAimController.Config flipped = cfg.copy();
        flipped.sign = -1;
        r = run(flipped, new Plant(0, 30, 120, -1, 3, 0.2), 8.0, 0);
        check(Math.abs(r.finalError) < 1.0, "sign -1 works on an inverted turret (" + r.finalError + ")");
        r = run(cfg, new Plant(0, 30, 120, -1, 3, 0.2), 3.0, 0);
        check(Math.abs(r.finalError) > 30.0, "wrong sign diverges instead of hiding the mistake");

        // 4. Aim offset trims where tx settles.
        VisionAimController.Config trimmed = cfg.copy();
        trimmed.aimOffsetDeg = 2.0;
        r = run(trimmed, new Plant(0, 10, 120, +1, 3, 0.1), 8.0, 2.0);
        check(Math.abs(r.finalError) < 1.0, "converges to the aim offset (" + r.finalError + ")");

        // 5. Deadband: no power for tiny error, even with noise below the deadband.
        VisionAimController aim = new VisionAimController(cfg);
        for (int i = 0; i < 50; i++) {
            VisionAimController.Output o = aim.update(true, 0.3, 0.02);
            check(o.demand == 0, "zero demand inside deadband");
        }
        check(aim.update(true, 0.3, 0.02).locked, "locks inside deadband");

        // 6. Lost target: zero demand immediately, filter resets, no derivative kick on reacquire.
        aim = new VisionAimController(cfg);
        for (int i = 0; i < 20; i++) aim.update(true, 10.0, 0.02);
        VisionAimController.Output lost = aim.update(false, Double.NaN, 0.02);
        check(lost.demand == 0 && !lost.targetVisible && lost.state.equals("SEARCHING"), "lost target holds with zero demand");
        for (int i = 0; i < 20; i++) aim.update(false, Double.NaN, 0.02);
        VisionAimController.Output back = aim.update(true, -20.0, 0.02);
        check(back.errorRateDegPerSec == 0, "reacquire after a long loss has no stale derivative");
        check(back.demand < 0, "reacquire aims toward the new target");

        // 7. Garbage input never produces output.
        aim = new VisionAimController(cfg);
        check(aim.update(true, Double.NaN, 0.02).demand == 0, "NaN tx gives zero demand");
        check(aim.update(true, Double.POSITIVE_INFINITY, 0.02).demand == 0, "infinite tx gives zero demand");
        check(Math.abs(aim.update(true, 90.0, 0.02).demand) <= cfg.maxPower, "huge error is capped");

        // 8. Invalid configuration fails closed.
        VisionAimController.Config bad = cfg.copy();
        bad.maxPower = 5.0;
        check(new VisionAimController(bad).update(true, 20, 0.02).state.equals("BAD_CONFIG"), "maxPower > 1 rejected");
        bad = cfg.copy();
        bad.sign = 0;
        check(new VisionAimController(bad).update(true, 20, 0.02).demand == 0, "sign 0 rejected");
        bad = cfg.copy();
        bad.kP = Double.NaN;
        check(new VisionAimController(bad).update(true, 20, 0.02).demand == 0, "NaN gain rejected");

        // 9. Hitches in dt cannot create a huge derivative.
        aim = new VisionAimController(cfg);
        aim.update(true, 10, 0.02);
        VisionAimController.Output hitch = aim.update(true, 9, 5.0);
        check(Math.abs(hitch.demand) <= cfg.maxPower, "long dt hitch stays capped");

        System.out.println("VISION AIM CHECKS PASSED (" + checks + ")");
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }
}
