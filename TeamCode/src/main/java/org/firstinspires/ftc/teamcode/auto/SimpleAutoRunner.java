package org.firstinspires.ftc.teamcode.auto;

import java.util.List;

/**
 * Runs a {@link SimpleAutoPlan} one loop at a time. Pure logic, no SDK.
 *
 * <p>Each loop the OpMode passes the elapsed time and whether the flywheel is at speed, and gets back
 * what to command. Guarantees:
 * <ul>
 *   <li>Everything is commanded off at {@code totalSec - endReserveSec}.</li>
 *   <li>No new shot sequence starts after {@code noNewShotAfterSec}.</li>
 *   <li>At {@code parkByElapsedSec} the remaining steps are abandoned and the robot goes straight to parking.</li>
 *   <li>A shot never waits forever: spin-up and recovery both time out, and the sequence continues.</li>
 *   <li>Drive power ramps up at {@code slewPerSec} and drops to zero immediately.</li>
 * </ul>
 */
public final class SimpleAutoRunner {
    public static final class Params {
        public double totalSec = 30.0;
        public double endReserveSec = 0.3;
        public double parkByElapsedSec = 25.5;
        public double noNewShotAfterSec = 24.0;
        public double slewPerSec = 3.0;
        public double spinupMaxSec = 4.0;
        public double readyHoldSec = 0.20;
        public double feedPulseSec = 0.35;
        public double minRecoverSec = 0.15;
        public double recoverMaxSec = 1.5;
        public double finalSettleSec = 0.30;
        public double maxDt = 0.25;
    }

    public static final class Command {
        public double x, y, yaw;
        public boolean flywheelOn, feed, intake, done;
        public String label = "";
        /** What the current step is doing right now. */
        public String status = "";
        /** Most recent notable event (timeouts, skipped steps). Sticks until the next one. */
        public String event = "";
    }

    private enum Phase { SPINUP, FEED, RECOVER, SETTLE }

    private final List<SimpleAutoPlan.Step> steps;
    private final Params p;
    private final int parkIndex;

    private double elapsed;
    private int index;
    private double stepElapsed;
    private boolean stepEntered;
    /** Drive legs end when the commanded power, integrated over time, reaches the leg length (full-power seconds). */
    private double driveProgress;
    private double forwardEquivSec, strafeEquivSec;

    private Phase phase;
    private double phaseElapsed;
    private double readyFor;
    private int shotsFired;

    private double outX, outY, outYaw;
    private boolean finished;
    private String lastStatus = "";
    private boolean abandoned;

    public SimpleAutoRunner(List<SimpleAutoPlan.Step> steps, Params params) {
        if (steps == null || params == null) throw new IllegalArgumentException("steps and params required");
        if (!(params.totalSec > 0) || !(params.slewPerSec > 0) || !(params.maxDt > 0)
                || Double.isInfinite(params.totalSec) || Double.isInfinite(params.slewPerSec) || Double.isInfinite(params.maxDt)
                || Double.isNaN(params.endReserveSec) || Double.isNaN(params.parkByElapsedSec) || Double.isNaN(params.noNewShotAfterSec)) {
            throw new IllegalArgumentException("totalSec, slewPerSec and maxDt must be positive and finite");
        }
        this.steps = steps;
        this.p = params;
        int park = steps.size();
        for (int i = 0; i < steps.size(); i++) if (steps.get(i).park) { park = i; break; }
        this.parkIndex = park;
    }

    public double elapsed() { return elapsed; }
    public int shotsFired() { return shotsFired; }
    public int stepIndex() { return index; }
    public boolean abandoned() { return abandoned; }
    /** Total full-power-equivalent seconds driven along each axis so far. Used to calibrate speeds. */
    public double forwardEquivalentSeconds() { return forwardEquivSec; }
    public double strafeEquivalentSeconds() { return strafeEquivSec; }

    public Command update(double dt, boolean flywheelAtSpeed) {
        if (!(dt >= 0) || Double.isInfinite(dt)) dt = 0;
        if (dt > p.maxDt) dt = p.maxDt;
        elapsed += dt;
        stepElapsed += dt;

        Command out = new Command();
        if (finished || elapsed >= p.totalSec - p.endReserveSec) {
            finished = true;
            outX = outY = outYaw = 0;
            out.done = true;
            out.label = "DONE";
            out.status = "finished";
            out.event = lastStatus;
            return out;
        }

        if (!abandoned && elapsed >= p.parkByElapsedSec && index < parkIndex) {
            abandoned = true;
            lastStatus = "RUNNING OUT OF TIME: going to park";
            gotoStep(parkIndex);
        }

        // Skip steps that can no longer run, then enter the current step.
        while (index < steps.size()) {
            SimpleAutoPlan.Step s = steps.get(index);
            if (s.kind == SimpleAutoPlan.Kind.SHOOT && !stepEntered && elapsed >= p.noNewShotAfterSec) {
                lastStatus = "SKIPPED " + s.label + " (too late)";
                gotoStep(index + 1);
                continue;
            }
            if (!stepEntered) enterStep(s);
            break;
        }

        if (index >= steps.size()) {
            finished = true;
            outX = outY = outYaw = 0;
            out.done = true;
            out.label = "DONE";
            out.status = "plan complete";
            out.event = lastStatus;
            return out;
        }

        SimpleAutoPlan.Step s = steps.get(index);
        double tx = 0, ty = 0, tyaw = 0;
        out.label = s.label;
        out.intake = s.intake;

        switch (s.kind) {
            case WAIT:
                if (stepElapsed >= s.seconds) { gotoStep(index + 1); return continueNext(dt, flywheelAtSpeed, out); }
                break;
            case DRIVE: {
                applySlew(s.x, s.y, s.yaw, dt, out);
                double mag = Math.max(Math.abs(out.x), Math.max(Math.abs(out.y), Math.abs(out.yaw)));
                driveProgress += mag * dt;
                forwardEquivSec += Math.abs(out.y) * dt;
                strafeEquivSec += Math.abs(out.x) * dt;
                // The second clause cannot normally trigger; it stops a drive that somehow never progresses.
                if (driveProgress >= s.seconds || stepElapsed >= s.seconds + 3.0 / p.slewPerSec + 1.0) {
                    outX = outY = outYaw = 0;
                    gotoStep(index + 1);
                    return continueNext(dt, flywheelAtSpeed, out);
                }
                out.flywheelOn = flywheelWanted();
                out.event = lastStatus;
                return out;
            }
            case SHOOT:
                if (runShoot(s, dt, flywheelAtSpeed, out)) { gotoStep(index + 1); return continueNext(dt, flywheelAtSpeed, out); }
                break;
            default:
                break;
        }

        applySlew(tx, ty, tyaw, dt, out);
        out.flywheelOn = flywheelWanted();
        out.event = lastStatus;
        return out;
    }

    /** A step ended this loop: run the next one on the same loop so no 20 ms gap is wasted. */
    private Command continueNext(double dt, boolean atSpeed, Command previous) {
        if (index >= steps.size()) {
            finished = true;
            outX = outY = outYaw = 0;
            Command done = new Command();
            done.done = true;
            done.label = "DONE";
            done.status = "plan complete";
            done.event = lastStatus;
            return done;
        }
        return updateNoTime(atSpeed);
    }

    private Command updateNoTime(boolean atSpeed) {
        // Re-enter with zero time so the new step is set up and its first command is produced.
        return update(0, atSpeed);
    }

    private void gotoStep(int next) {
        index = next;
        stepElapsed = 0;
        stepEntered = false;
    }

    private void enterStep(SimpleAutoPlan.Step s) {
        stepEntered = true;
        stepElapsed = 0;
        driveProgress = 0;
        if (s.kind == SimpleAutoPlan.Kind.SHOOT) {
            phase = Phase.SPINUP;
            phaseElapsed = 0;
            readyFor = 0;
            shotsFired = 0;
        }
    }

    /** @return true when this shoot step is finished. */
    private boolean runShoot(SimpleAutoPlan.Step s, double dt, boolean atSpeed, Command out) {
        phaseElapsed += dt;
        switch (phase) {
            case SPINUP:
                readyFor = atSpeed ? readyFor + dt : 0;
                if (readyFor >= p.readyHoldSec) {
                    startFeed();
                } else if (phaseElapsed >= p.spinupMaxSec) {
                    lastStatus = "SPIN-UP TIMEOUT: shooting anyway";
                    startFeed();
                } else {
                    out.status = "spinning up";
                }
                break;
            case FEED:
                out.feed = true;
                out.status = "feeding shot " + (shotsFired + 1) + "/" + s.shots;
                if (phaseElapsed >= p.feedPulseSec) {
                    shotsFired++;
                    phase = shotsFired >= s.shots ? Phase.SETTLE : Phase.RECOVER;
                    phaseElapsed = 0;
                    readyFor = 0;
                    out.feed = false;
                }
                break;
            case RECOVER:
                out.status = "recovering speed";
                if (phaseElapsed >= p.minRecoverSec && atSpeed) {
                    startFeed();
                } else if (phaseElapsed >= p.recoverMaxSec) {
                    lastStatus = "RECOVERY TIMEOUT: feeding next ball anyway";
                    startFeed();
                }
                break;
            case SETTLE:
                out.status = "letting last ball clear";
                if (phaseElapsed >= p.finalSettleSec) return true;
                break;
            default:
                break;
        }
        return false;
    }

    private void startFeed() {
        phase = Phase.FEED;
        phaseElapsed = 0;
    }

    /** Keep the flywheel on from the start until the last shoot step is done. */
    private boolean flywheelWanted() {
        if (finished) return false;
        for (int i = index; i < steps.size(); i++) {
            if (steps.get(i).kind == SimpleAutoPlan.Kind.SHOOT) return true;
        }
        return false;
    }

    private void applySlew(double tx, double ty, double tyaw, double dt, Command out) {
        outX = slew(outX, tx, dt);
        outY = slew(outY, ty, dt);
        outYaw = slew(outYaw, tyaw, dt);
        out.x = outX;
        out.y = outY;
        out.yaw = outYaw;
    }

    /** Ramp up at slewPerSec; drop to the target at once when it is smaller in magnitude or reverses. */
    private double slew(double current, double target, double dt) {
        if (target == 0) return 0;
        if (current != 0 && Math.signum(current) != Math.signum(target)) current = 0;
        if (Math.abs(target) <= Math.abs(current)) return target;
        double maxStep = p.slewPerSec * dt;
        double delta = target - current;
        return Math.abs(delta) <= maxStep ? target : current + Math.signum(delta) * maxStep;
    }
}
