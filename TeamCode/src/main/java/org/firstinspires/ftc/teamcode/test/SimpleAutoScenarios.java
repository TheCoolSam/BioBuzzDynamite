package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.auto.FlywheelReadyDetector;
import org.firstinspires.ftc.teamcode.auto.SimpleAutoPlan;
import org.firstinspires.ftc.teamcode.auto.SimpleAutoRunner;

import java.util.ArrayList;
import java.util.List;

/** Desktop checks for the Simple Auto: plan building, step running, deadlines, and the flywheel ready detector. */
public final class SimpleAutoScenarios {
    private static int checks;
    private static final double DT = 0.02;

    /** First-order flywheel with a dip each time a ball is fed. */
    private static final class SimFlywheel {
        double rpm;
        final double plateau;
        final double tau;
        boolean lastFeed;
        double noise;          // +/- fraction of random reading noise
        final java.util.Random rng = new java.util.Random(7);
        boolean dead;          // encoder reads zero
        boolean stuck;         // never speeds up
        SimFlywheel(double plateau, double tau) { this.plateau = plateau; this.tau = tau; }
        void step(double dt, boolean on, boolean feeding) {
            double target = on && !stuck ? plateau : 0;
            rpm += (target - rpm) * Math.min(1.0, dt / tau);
            if (feeding && !lastFeed && rpm > 100) rpm *= 0.90;
            lastFeed = feeding;
        }
        double reading() { return dead ? 0 : rpm * (1 + noise * (rng.nextDouble() * 2 - 1)); }
    }

    private static final class Run {
        double finishTime = Double.NaN;
        int pulses;
        final List<Double> speedAtPulse = new ArrayList<Double>();
        final List<String> labels = new ArrayList<String>();
        double maxIntakeOutsidePickup;
        boolean feedWhileMoving;
        boolean flywheelOnAtStart;
        double lastFlywheelOnTime = -1;
        double lastFeedTime = -1;
        boolean commandsZeroAtEnd;
        double maxAbsDrive;
        String lastEvent = "";
        boolean noSignal;
        double firstFeedTime = Double.NaN;
        double maxPowerStep;
    }

    private static Run simulate(SimpleAutoPlan.Config cfg, SimpleAutoRunner.Params rp, FlywheelReadyDetector.Config dc,
                                SimFlywheel fly) {
        List<SimpleAutoPlan.Step> steps = SimpleAutoPlan.build(cfg);
        SimpleAutoRunner runner = new SimpleAutoRunner(steps, rp);
        FlywheelReadyDetector det = new FlywheelReadyDetector(dc);
        Run r = new Run();
        double t = 0;
        boolean wasFeeding = false;
        double prevY = 0, prevX = 0;
        for (int i = 0; i < 3000; i++) {
            SimpleAutoRunner.Command c = runner.update(DT, det.ready());
            t += DT;
            if (c.done) {
                r.finishTime = t;
                r.commandsZeroAtEnd = c.x == 0 && c.y == 0 && c.yaw == 0 && !c.feed && !c.intake && !c.flywheelOn;
                r.lastEvent = c.event;
                break;
            }
            if (i == 0) r.flywheelOnAtStart = c.flywheelOn;
            if (c.flywheelOn) r.lastFlywheelOnTime = t;
            if (c.feed) {
                r.lastFeedTime = t;
                if (Double.isNaN(r.firstFeedTime)) r.firstFeedTime = t;
                if (c.x != 0 || c.y != 0 || c.yaw != 0) r.feedWhileMoving = true;
                if (!wasFeeding) {
                    r.pulses++;
                    r.speedAtPulse.add(fly.rpm / fly.plateau);
                }
            }
            wasFeeding = c.feed;
            if (c.intake && !c.label.startsWith("PICKUP")) r.maxIntakeOutsidePickup = 1;
            r.maxAbsDrive = Math.max(r.maxAbsDrive, Math.max(Math.abs(c.x), Math.abs(c.y)));
            r.maxPowerStep = Math.max(r.maxPowerStep, Math.max(Math.abs(c.y - prevY), Math.abs(c.x - prevX)));
            prevX = c.x; prevY = c.y;
            if (r.labels.isEmpty() || !r.labels.get(r.labels.size() - 1).equals(c.label)) r.labels.add(c.label);
            fly.step(DT, c.flywheelOn, c.feed);
            det.update(DT, fly.reading(), c.flywheelOn);
            r.noSignal |= det.noSignal();
        }
        return r;
    }

    private static FlywheelReadyDetector.Config openLoop() {
        FlywheelReadyDetector.Config c = new FlywheelReadyDetector.Config();
        c.mode = FlywheelReadyDetector.Mode.OPEN_LOOP;
        return c;
    }

    public static void main(String[] args) {
        validation();
        planShapes();
        detector();
        normalRoutes();
        failureModes();
        deadlines();
        safetyAndSlew();
        System.out.println("SIMPLE AUTO CHECKS PASSED (" + checks + ")");
    }

    // ------------------------------------------------------------------------------------------ validation
    private static void validation() {
        check(SimpleAutoPlan.validate(new SimpleAutoPlan.Config()).isEmpty(), "default config validates");
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config(); c.inPerSecForward = 0;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "zero forward speed rejected");
        c = new SimpleAutoPlan.Config(); c.inPerSecStrafe = Double.NaN;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "NaN strafe speed rejected");
        c = new SimpleAutoPlan.Config(); c.route = 3;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "route 3 rejected");
        c = new SimpleAutoPlan.Config(); c.route = -1;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "route -1 rejected");
        c = new SimpleAutoPlan.Config(); c.shots = 0;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "0 shots rejected");
        c = new SimpleAutoPlan.Config(); c.shots = 5;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "5 shots rejected (capacity 4)");
        c = new SimpleAutoPlan.Config(); c.parkForwardIn = 500;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "500 in leg rejected");
        c = new SimpleAutoPlan.Config(); c.parkForwardIn = Double.POSITIVE_INFINITY;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "infinite leg rejected");
        c = new SimpleAutoPlan.Config(); c.route = 0; c.parkForwardIn = 0; c.parkStrafeIn = 0;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "leave route with no distance rejected");
        c = new SimpleAutoPlan.Config(); c.route = 2; c.pickupForwardIn = 0; c.pickupStrafeIn = 0;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "pickup route with no distance rejected");
        c = new SimpleAutoPlan.Config(); c.startDelaySec = -1;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "negative delay rejected");
        c = new SimpleAutoPlan.Config(); c.startDelaySec = 99;
        check(!SimpleAutoPlan.validate(c).isEmpty(), "99 s delay rejected");
        try { SimpleAutoPlan.build(null); check(false, "null config must throw"); } catch (IllegalArgumentException expected) { check(true, "null config throws"); }
        c = new SimpleAutoPlan.Config(); c.shots = 0;
        try { SimpleAutoPlan.build(c); check(false, "bad config must not build"); } catch (IllegalArgumentException expected) { check(true, "bad config does not build"); }
    }

    // ------------------------------------------------------------------------------------------ plan shapes
    private static void planShapes() {
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();
        c.route = 0;
        List<SimpleAutoPlan.Step> s = SimpleAutoPlan.build(c);
        check(s.size() == 1 && s.get(0).park && s.get(0).kind == SimpleAutoPlan.Kind.DRIVE, "leave route is one parking drive");
        check(Math.abs(s.get(0).seconds - c.parkForwardIn / c.inPerSecForward) < 1e-9, "drive time = inches / measured speed");
        check(s.get(0).y == 1 && s.get(0).x == 0, "forward leg drives +Y only");

        c = new SimpleAutoPlan.Config(); c.route = 1;
        s = SimpleAutoPlan.build(c);
        check(count(s, SimpleAutoPlan.Kind.SHOOT) == 1, "preload route has one shoot step");
        check(s.get(0).kind == SimpleAutoPlan.Kind.SHOOT, "no shot move when distance is zero");
        check(s.get(s.size() - 1).park, "plan ends with a parking leg");

        c = new SimpleAutoPlan.Config(); c.route = 2; c.pickupStrafeIn = 10; c.pickupForwardIn = 20;
        s = SimpleAutoPlan.build(c);
        check(count(s, SimpleAutoPlan.Kind.SHOOT) == 2, "pickup route shoots twice");
        double out = 0, back = 0;
        boolean intakeOnlyInPickup = true;
        for (SimpleAutoPlan.Step st : s) {
            if (st.label.startsWith("PICKUP OUT")) out += st.seconds;
            if (st.label.startsWith("PICKUP BACK")) back += st.seconds;
            if (st.intake != st.label.startsWith("PICKUP")) intakeOnlyInPickup = false;
        }
        check(Math.abs(out - back) < 1e-9, "pickup return takes as long as the way out");
        check(intakeOnlyInPickup, "intake is on only during pickup steps");
        double outX = 0, outY = 0, backX = 0, backY = 0;
        for (SimpleAutoPlan.Step st : s) {
            if (st.label.startsWith("PICKUP OUT")) { outX += st.x * st.seconds; outY += st.y * st.seconds; }
            if (st.label.startsWith("PICKUP BACK")) { backX += st.x * st.seconds; backY += st.y * st.seconds; }
        }
        check(Math.abs(outX + backX) < 1e-9 && Math.abs(outY + backY) < 1e-9, "pickup return exactly retraces the way out");

        // Blue mirrors every strafe.
        c = new SimpleAutoPlan.Config(); c.route = 2; c.parkStrafeIn = 12; c.pickupStrafeIn = 8; c.shotMoveStrafeIn = 6;
        List<SimpleAutoPlan.Step> red = SimpleAutoPlan.build(c);
        c.blue = true;
        List<SimpleAutoPlan.Step> blue = SimpleAutoPlan.build(c);
        check(red.size() == blue.size(), "red and blue have the same steps");
        boolean mirrored = true, sawStrafe = false;
        for (int i = 0; i < red.size(); i++) {
            if (red.get(i).x != 0) sawStrafe = true;
            if (red.get(i).x != -blue.get(i).x || red.get(i).y != blue.get(i).y
                    || Math.abs(red.get(i).seconds - blue.get(i).seconds) > 1e-9) mirrored = false;
        }
        check(sawStrafe && mirrored, "blue mirrors strafe and leaves forward and timing alone");

        // Delay becomes a step only when set.
        c = new SimpleAutoPlan.Config(); c.startDelaySec = 2;
        s = SimpleAutoPlan.build(c);
        check(s.get(0).kind == SimpleAutoPlan.Kind.WAIT && s.get(0).seconds == 2, "start delay is the first step");

        // Time budget estimates.
        SimpleAutoRunner.Params p = new SimpleAutoRunner.Params();
        c = new SimpleAutoPlan.Config(); c.route = 2;
        s = SimpleAutoPlan.build(c);
        double best = SimpleAutoPlan.bestCaseSeconds(s, p), worst = SimpleAutoPlan.worstCaseSeconds(s, p);
        check(best > 0 && worst > best, "worst case is slower than best case");
        check(worst < 30, "default two-shoot route fits in 30 s even if every timeout hits (" + worst + ")");
    }

    // ------------------------------------------------------------------------------------------ detector
    private static void detector() {
        // Open loop: ramp is not steady, plateau is.
        FlywheelReadyDetector.Config dc = openLoop();
        FlywheelReadyDetector d = new FlywheelReadyDetector(dc);
        SimFlywheel f = new SimFlywheel(4800, 0.7);
        boolean readyDuringRamp = false;
        double t = 0, readyAt = -1;
        for (int i = 0; i < 600; i++) {
            d.update(DT, f.reading(), true);
            f.step(DT, true, false);
            t += DT;
            if (d.ready() && readyAt < 0) readyAt = t;
            if (t < 1.5 && d.ready()) readyDuringRamp = true;
        }
        check(!readyDuringRamp, "open loop is not ready while still accelerating");
        check(readyAt > 1.5 && readyAt < 6, "open loop becomes ready once the plateau is steady (" + readyAt + ")");
        check(d.baselineRpm() > 4500 && d.baselineRpm() <= 4800, "baseline is the plateau rpm (" + d.baselineRpm() + ")");

        // Dip: not ready, then ready again after recovery.
        f.rpm *= 0.85;
        d.update(DT, f.reading(), true);
        check(d.ready(), "a single dipped sample does not flip readiness (smoothing)");
        for (int i = 0; i < 10; i++) d.update(DT, f.reading(), true); // dip held for 0.2 s
        check(!d.ready(), "not ready while a 15% dip persists");
        boolean recovered = false;
        for (int i = 0; i < 300 && !recovered; i++) {
            f.step(DT, true, false);
            d.update(DT, f.reading(), true);
            recovered = d.ready();
        }
        check(recovered, "ready again after the dip recovers");

        // Turning the flywheel off resets learning.
        d.update(DT, 0, false);
        check(!d.ready() && Double.isNaN(d.baselineRpm()), "switching off resets the detector");

        // Velocity mode.
        FlywheelReadyDetector.Config vc = new FlywheelReadyDetector.Config();
        vc.mode = FlywheelReadyDetector.Mode.VELOCITY; vc.targetRpm = 3000; vc.tolerance = 0.03;
        FlywheelReadyDetector v = new FlywheelReadyDetector(vc);
        for (int i = 0; i < 60; i++) v.update(DT, 2950, true);
        check(v.ready(), "velocity mode ready within 3% of target");
        for (int i = 0; i < 20; i++) v.update(DT, 2800, true);
        check(!v.ready(), "velocity mode not ready 6.7% low");
        for (int i = 0; i < 20; i++) v.update(DT, 3200, true);
        check(!v.ready(), "velocity mode not ready 6.7% high");

        // Timed mode.
        FlywheelReadyDetector.Config tc = new FlywheelReadyDetector.Config();
        tc.mode = FlywheelReadyDetector.Mode.TIMED; tc.timedSpinupSec = 2.0;
        FlywheelReadyDetector td = new FlywheelReadyDetector(tc);
        for (int i = 0; i < 90; i++) td.update(DT, 0, true);
        check(!td.ready(), "timed mode not ready before the time (1.8 s)");
        for (int i = 0; i < 20; i++) td.update(DT, 0, true);
        check(td.ready(), "timed mode ready after the time");

        // No encoder signal falls back to timed instead of hanging.
        FlywheelReadyDetector dead = new FlywheelReadyDetector(openLoop());
        for (int i = 0; i < 400; i++) dead.update(DT, 0, true);
        check(dead.noSignal() && dead.ready(), "dead encoder: flagged and falls back to the timer");

        // Garbage input never makes it ready by itself.
        FlywheelReadyDetector g = new FlywheelReadyDetector(openLoop());
        for (int i = 0; i < 20; i++) g.update(DT, Double.NaN, true);
        check(!g.ready(), "NaN rpm is not ready");
        g.update(Double.NaN, Double.POSITIVE_INFINITY, true);
        check(!g.ready(), "infinite rpm / dt is not ready");
    }

    // ------------------------------------------------------------------------------------------ normal runs
    private static void normalRoutes() {
        SimpleAutoRunner.Params rp = new SimpleAutoRunner.Params();
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();

        c.route = 0;
        Run r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.finishTime > 0.9 && r.finishTime < 3.2, "leave route finishes quickly (" + r.finishTime + ")");
        check(!r.flywheelOnAtStart && r.pulses == 0, "leave route never spins the flywheel or feeds");
        check(r.commandsZeroAtEnd, "everything is off when done");

        c.route = 1;
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.finishTime < 30, "preload route finishes inside 30 s (" + r.finishTime + ")");
        check(r.pulses == 3, "preload route fires 3 pulses (got " + r.pulses + ")");
        check(r.flywheelOnAtStart, "flywheel spins from the first loop");
        check(minOf(r.speedAtPulse) >= 0.93, "every pulse starts at/near full speed (min " + minOf(r.speedAtPulse) + ")");
        check(!r.feedWhileMoving, "robot is stationary while feeding");
        check(r.lastFlywheelOnTime <= r.lastFeedTime + rp.finalSettleSec + 0.1, "flywheel switches off right after the last shot");
        check(r.commandsZeroAtEnd, "everything is off when done");

        c.route = 2;
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.finishTime < 30, "pickup route finishes inside 30 s (" + r.finishTime + ")");
        check(r.pulses == 6, "pickup route fires 6 pulses (got " + r.pulses + ")");
        check(minOf(r.speedAtPulse) >= 0.93, "pickup route: every pulse at/near full speed (min " + minOf(r.speedAtPulse) + ")");
        check(r.maxIntakeOutsidePickup == 0, "intake only runs during pickup");
        check(!r.feedWhileMoving, "pickup route: stationary while feeding");
        check(r.commandsZeroAtEnd, "pickup route: everything off at the end");

        // Velocity-mode flywheel.
        FlywheelReadyDetector.Config vc = new FlywheelReadyDetector.Config();
        vc.mode = FlywheelReadyDetector.Mode.VELOCITY; vc.targetRpm = 4800;
        c.route = 1;
        r = simulate(c, rp, vc, new SimFlywheel(4800, 0.5));
        check(r.pulses == 3 && r.finishTime < 30, "velocity-mode preload route works");

        // Four balls.
        c.shots = 4;
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.pulses == 4, "4-shot route fires 4 pulses");

        // Noisy encoder velocity (+/-2%) must not stop it from finding the plateau.
        c.shots = 3;
        SimFlywheel noisy = new SimFlywheel(4800, 0.7); noisy.noise = 0.02;
        r = simulate(c, rp, openLoop(), noisy);
        check(r.pulses == 3 && r.finishTime < 30, "noisy encoder: still completes (pulses " + r.pulses + ", finish " + r.finishTime + ", speeds " + r.speedAtPulse + ", firstFeed " + r.firstFeedTime + ", event " + r.lastEvent + ")");
        check(minOf(r.speedAtPulse) >= 0.93, "noisy encoder: shots at/near full speed (min " + minOf(r.speedAtPulse) + ")");
        check(r.firstFeedTime < 4.0, "noisy encoder: did not just wait out the spin-up timeout (" + r.firstFeedTime + ")");

        // Slower flywheel (tau 1.0 s): still waits for speed instead of shooting early.
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 1.0));
        check(r.pulses == 3 && minOf(r.speedAtPulse) >= 0.93, "slow flywheel: still waits for speed (speeds " + r.speedAtPulse + ")");

        // Much slower flywheel (tau 1.4 s) cannot reach plateau inside the spin-up limit. It must still
        // finish, and it must tell the operator it gave up waiting.
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 1.4));
        check(r.pulses == 3 && r.finishTime < 30, "very slow flywheel: still completes");
        check(r.lastEvent.contains("TIMEOUT"), "very slow flywheel: timeout is reported (" + r.lastEvent + ")");
    }

    // ------------------------------------------------------------------------------------------ failure modes
    private static void failureModes() {
        SimpleAutoRunner.Params rp = new SimpleAutoRunner.Params();
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();
        c.route = 2;

        SimFlywheel dead = new SimFlywheel(4800, 0.7); dead.dead = true;
        Run r = simulate(c, rp, openLoop(), dead);
        check(r.noSignal, "dead encoder is detected");
        check(r.pulses == 6 && r.finishTime < 30, "dead encoder: auto still completes all shots (" + r.finishTime + ")");
        check(r.firstFeedTime >= 3.0, "dead encoder: waited for the timed spin-up (" + r.firstFeedTime + ")");

        SimFlywheel stuck = new SimFlywheel(4800, 0.7); stuck.stuck = true;
        r = simulate(c, rp, openLoop(), stuck);
        check(r.finishTime < 30, "flywheel that never spins up cannot hang the auto (" + r.finishTime + ")");
        check(r.commandsZeroAtEnd, "stuck flywheel: everything off at the end");
        check(r.noSignal, "stuck flywheel looks like a dead encoder and is flagged as NO SIGNAL");

        // Detector that never gets ready in velocity mode at an unreachable target.
        FlywheelReadyDetector.Config vc = new FlywheelReadyDetector.Config();
        vc.mode = FlywheelReadyDetector.Mode.VELOCITY; vc.targetRpm = 9000;
        r = simulate(c, rp, vc, new SimFlywheel(4800, 0.7));
        check(r.finishTime < 30, "unreachable velocity target cannot hang the auto (" + r.finishTime + ")");
        check(r.pulses > 0, "unreachable target: shoots after the spin-up timeout");
    }

    // ------------------------------------------------------------------------------------------ deadlines
    private static void deadlines() {
        SimpleAutoRunner.Params rp = new SimpleAutoRunner.Params();
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();

        // A long start delay pushes the shot late. The robot must still park and stop before 30 s.
        c.route = 2; c.startDelaySec = 15; c.pickupForwardIn = 90; c.parkForwardIn = 40;
        Run r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.finishTime < 30, "late plan still ends before 30 s (" + r.finishTime + ")");
        check(r.pulses < 6, "late plan drops the second shot");
        check(r.labels.get(r.labels.size() - 1).startsWith("PARK"), "late plan ends by parking (last: " + r.labels.get(r.labels.size() - 1) + ")");
        check(r.commandsZeroAtEnd, "late plan: everything off at the end");

        // Beyond the cut-off no shot sequence starts at all.
        c = new SimpleAutoPlan.Config();
        c.route = 1; c.startDelaySec = 14.9;
        // A 144 in move takes 4.8 s, so the robot arrives at about 19.7 s... push it further with a slow speed.
        c.inPerSecForward = 12; // 144 in takes 12 s: arrives at ~26.9 s, after the 24 s cut-off
        c.shotMoveForwardIn = 144;
        c.parkForwardIn = 6;
        r = simulate(c, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.pulses == 0, "arriving after the cut-off skips the shot (" + r.pulses + " pulses)");
        check(r.finishTime < 30, "skipped-shot plan still ends before 30 s (" + r.finishTime + ")");
        check(r.lastEvent.contains("SKIPPED") || r.lastEvent.contains("OUT OF TIME"), "skip is reported: " + r.lastEvent);

        // Park-by deadline abandons a shot in progress.
        c = new SimpleAutoPlan.Config();
        c.route = 1; c.startDelaySec = 15; c.parkForwardIn = 30;
        SimFlywheel stuck = new SimFlywheel(4800, 0.7); stuck.stuck = true;
        SimpleAutoRunner.Params slow = new SimpleAutoRunner.Params();
        slow.spinupMaxSec = 12; // a very patient spin-up, to prove the park-by deadline wins
        r = simulate(c, slow, openLoop(), stuck);
        check(r.finishTime < 30, "park-by deadline overrides a long spin-up (" + r.finishTime + ")");
        check(r.labels.get(r.labels.size() - 1).startsWith("PARK"), "abandoned shot still ends by parking");
        check(r.lastFeedTime < 0 || r.lastFeedTime < slow.parkByElapsedSec, "no feeding after the park-by deadline");
    }

    // ------------------------------------------------------------------------------------------ safety and slew
    private static void safetyAndSlew() {
        SimpleAutoRunner.Params rp = new SimpleAutoRunner.Params();
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();
        c.route = 0; c.parkForwardIn = 60;
        SimpleAutoRunner runner = new SimpleAutoRunner(SimpleAutoPlan.build(c), rp);
        SimpleAutoRunner.Command first = runner.update(DT, false);
        check(Math.abs(first.y) <= rp.slewPerSec * DT + 1e-9, "first drive command ramps up (y=" + first.y + ")");
        SimpleAutoRunner.Command cmd = first;
        double t = DT;
        while (cmd.y < 1.0 && t < 2) { cmd = runner.update(DT, false); t += DT; }
        check(Math.abs(t - 1.0 / rp.slewPerSec) < 3 * DT, "reaches full drive power after 1/slew s (" + t + ")");
        // Run to the end of the leg: the command must go straight to zero, with no tail.
        double prev = cmd.y;
        boolean droppedAtOnce = false;
        for (int i = 0; i < 400; i++) {
            cmd = runner.update(DT, false);
            if (cmd.y < prev - 1e-9 && cmd.y != 0) { droppedAtOnce = false; break; }
            if (prev == 1.0 && cmd.y == 0) { droppedAtOnce = true; break; }
            prev = cmd.y;
        }
        check(droppedAtOnce, "drive power drops straight to zero at the end of a leg");

        // Distance accounting: a 60 in leg at 30 in/s must integrate to exactly 2.0 full-power seconds,
        // even though the ramp-up makes the leg last a little longer than 2.0 s of wall time.
        SimpleAutoRunner dist = new SimpleAutoRunner(SimpleAutoPlan.build(c), rp);
        double wall = 0, integral = 0;
        for (int i = 0; i < 3000; i++) {
            SimpleAutoRunner.Command k = dist.update(DT, false);
            if (k.done) break;
            wall += DT;
            integral += Math.abs(k.y) * DT;
        }
        check(Math.abs(integral - 2.0) <= 0.05, "leg integrates to length / speed (" + integral + ")");
        check(wall > 2.0, "ramp-up makes the leg last longer than length/speed of wall time (" + wall + ")");
        check(Math.abs(dist.forwardEquivalentSeconds() - 2.0) <= 0.05, "runner reports forward full-power seconds (" + dist.forwardEquivalentSeconds() + ")");

        // Bad parameters are refused.
        SimpleAutoRunner.Params bad = new SimpleAutoRunner.Params(); bad.slewPerSec = 0;
        try { new SimpleAutoRunner(SimpleAutoPlan.build(c), bad); check(false, "zero slew must be refused"); }
        catch (IllegalArgumentException expected) { check(true, "zero slew refused"); }
        bad = new SimpleAutoRunner.Params(); bad.totalSec = Double.NaN;
        try { new SimpleAutoRunner(SimpleAutoPlan.build(c), bad); check(false, "NaN total must be refused"); }
        catch (IllegalArgumentException expected) { check(true, "NaN total refused"); }

        // Hostile dt values do not break time keeping.
        runner = new SimpleAutoRunner(SimpleAutoPlan.build(new SimpleAutoPlan.Config()), rp);
        runner.update(Double.NaN, false);
        check(runner.elapsed() == 0, "NaN dt adds no time");
        runner.update(-5, false);
        check(runner.elapsed() == 0, "negative dt adds no time");
        runner.update(Double.POSITIVE_INFINITY, false);
        check(runner.elapsed() == 0, "infinite dt adds no time");
        runner.update(100, false);
        check(runner.elapsed() <= rp.maxDt + 1e-9, "a huge dt is capped at maxDt");

        // After done it stays done and silent.
        runner = new SimpleAutoRunner(SimpleAutoPlan.build(c), rp);
        SimpleAutoRunner.Command last = null;
        for (int i = 0; i < 3000; i++) { last = runner.update(DT, false); if (last.done) break; }
        check(last != null && last.done, "run completes");
        SimpleAutoRunner.Command after = runner.update(DT, true);
        check(after.done && after.y == 0 && !after.feed && !after.flywheelOn && !after.intake, "stays done and off after finishing");

        // Hard stop at total time even if the plan is far longer.
        SimpleAutoPlan.Config longPlan = new SimpleAutoPlan.Config();
        longPlan.route = 0; longPlan.parkForwardIn = 144; longPlan.inPerSecForward = 1; // 144 s leg
        Run r = simulate(longPlan, rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.finishTime <= rp.totalSec - rp.endReserveSec + 0.1, "hard stop at 30 s minus reserve (" + r.finishTime + ")");
        check(r.commandsZeroAtEnd, "hard stop: everything off");

        // Driving is never commanded beyond the plan's unit magnitude.
        r = simulate(new SimpleAutoPlan.Config(), rp, openLoop(), new SimFlywheel(4800, 0.7));
        check(r.maxAbsDrive <= 1.0, "drive command never exceeds 1.0");
        check(r.maxPowerStep <= rp.slewPerSec * DT + 1e-9 || r.maxPowerStep == 0 || r.maxPowerStep <= 1.0,
                "power only steps down all at once");
    }

    // ------------------------------------------------------------------------------------------ helpers
    private static int count(List<SimpleAutoPlan.Step> steps, SimpleAutoPlan.Kind kind) {
        int n = 0;
        for (SimpleAutoPlan.Step s : steps) if (s.kind == kind) n++;
        return n;
    }

    private static double minOf(List<Double> xs) {
        double m = Double.MAX_VALUE;
        for (double x : xs) m = Math.min(m, x);
        return xs.isEmpty() ? 0 : m;
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }
}
