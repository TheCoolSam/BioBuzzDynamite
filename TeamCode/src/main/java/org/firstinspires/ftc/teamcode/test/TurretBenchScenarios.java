package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.turret.*;
import java.lang.reflect.Field;

/** No-hardware tests of the same conversion, operator, and final output gates used on the Hub. */
public final class TurretBenchScenarios {
    private static int checks;
    public static void main(String[] args) throws Exception {
        TurretOutputSafety gate = gate();
        for (double bad : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            near(gate.apply(.8, bad, false, .02, true, true, false), 0, "T1 invalid angle");
            near(gate.apply(bad, 0, true, .02, true, true, false), 0, "invalid command");
        }
        near(gate.apply(.8, 1, true, .02, true, true, false), 0, "T2 positive stop");
        near(gate.apply(-.1, 1, true, .02, true, true, false), -.1, "T2 inward allowed");
        near(gate.apply(-.1, 1.1, true, .02, true, true, false), -.1, "T2 beyond bench inward allowed");
        near(gate.apply(-.8, -1, true, .02, true, true, false), 0, "T3 negative stop");
        near(gate.apply(.1, -1, true, .02, true, true, false), .1, "T3 inward allowed");
        near(gate.apply(.1, -1.1, true, .02, true, true, false), .1, "T3 beyond bench inward allowed");
        near(gate.apply(.1, -2.1, true, .02, true, true, false), 0, "impossible physical angle blocked");
        near(gate.apply(.8, 0, true, .02, true, true, false), .2, "T4 cap");
        near(gate.apply(-.8, 0, true, .02, true, true, false), -.2, "negative cap");
        near(gate.apply(.1, 0, true, .02, true, false, false), 0, "T5 deadman");
        near(gate.apply(.1, 0, true, .02, false, true, false), 0, "unarmed");
        near(gate.apply(.1, 0, true, .02, true, true, true), 0, "estop");
        near(new TurretOutputSafety(-2, 2, -1.5, 1.5, -3, 3, .2)
                .apply(.1, 0, true, .02, true, true, false), 0, "bad nesting");
        conversion();
        timingAndControllerReset();
        operator();
        stopping();
        System.out.println("TURRET BENCH CHECKS PASSED (" + checks + ")");
    }
    private static TurretOutputSafety gate() {
        return new TurretOutputSafety(-2, 2, -1.5, 1.5, -1, 1, .2);
    }
    private static AbsoluteTurretSensor.Reading reading(double degrees) {
        double turns = degrees / 360;
        turns -= Math.floor(turns);
        return new AbsoluteTurretSensor.Reading(turns, "turns (fixture)", turns, "");
    }
    private static void update(TurretEncoderSample s, double deg, int sign, double offset) {
        s.update(reading(deg), .02, sign, offset, true, 10);
    }
    private static TurretEncoderSample valid(double deg) {
        TurretEncoderSample s = new TurretEncoderSample();
        update(s, deg, 1, 0); update(s, deg, 1, 0); return s;
    }
    private static void conversion() {
        for (double deg : new double[] {-120, -45, 60, 140, 80, 10}) {
            TurretEncoderSample s = new TurretEncoderSample();
            update(s, deg, 1, 0);
            near(s.angleRad, Math.toRadians(deg), "absolute startup position " + deg);
            check(!s.valid && Double.isNaN(s.velocityRadPerSec), "first loop needs velocity baseline");
            update(s, deg, 1, 0);
            check(s.valid, "second sample ready without homing");
            near(s.velocityRadPerSec, 0, "stationary startup velocity");
        }
        TurretEncoderSample s = valid(10);
        update(s, 11, 1, 0);
        check(s.angleRad > Math.toRadians(10) && s.velocityRadPerSec > 0, "T6 CCW contract");
        s = new TurretEncoderSample();
        update(s, 40, -1, Math.toRadians(-30));
        near(s.angleRad, Math.toRadians(-10), "encoder direction and offset");
        s = new TurretEncoderSample();
        update(s, 359, 1, 0); update(s, 1, 1, 0);
        near(s.velocityRadPerSec, Math.toRadians(2) / .02, "raw sensor wrap has no fake spike");
        s = new TurretEncoderSample();
        update(s, 80, 1, Double.NaN);
        check(!s.valid && Double.isNaN(s.angleRad) && s.fault.equals("NOT CALIBRATED"), "no invented zero");
        near(s.unoffsetRad, Math.toRadians(80), "calibration suggestion survives");
        s.update(reading(0), .02, 1, 0, false, 10);
        check(!s.valid, "motor-side ambiguous angle blocked");
        s.update(AbsoluteTurretSensor.Reading.invalid("READ FAILURE"), .02, 1, 0, true, 10);
        check(!s.valid && Double.isNaN(s.angleRad), "read failure invalidates angle");
        s.update(new AbsoluteTurretSensor.Reading(1, "turns", 1, ""), .02, 1, 0, true, 10);
        check(!s.valid, "turns range rejected");
        s = valid(0); update(s, 100, 1, 0);
        check(!s.valid && Double.isNaN(s.velocityRadPerSec), "impossible rate rejected");
        update(s, 100, 1, 0);
        check(!s.valid && s.fault.equals("VELOCITY BASELINE REQUIRED"), "fault clears velocity history");
        update(s, 100, 1, 0); check(s.valid, "invalid to valid baseline recovers");
    }
    private static TurretState aim(double bearing, double measured) {
        TurretState s = new TurretState();
        s.targetX = Math.cos(Math.toRadians(bearing)); s.targetY = Math.sin(Math.toRadians(bearing));
        s.turretAngle = Math.toRadians(measured); s.dt = .02; return s;
    }
    private static void timingAndControllerReset() throws Exception {
        TurretController controller = new TurretController();
        Field memory = TurretController.class.getDeclaredField("hasLastError");
        memory.setAccessible(true);
        controller.calculate(aim(179, 150));
        check(memory.getBoolean(controller), "active derivative baseline exists");
        TurretState invalid = aim(0, 0); invalid.turretAngle = Double.NaN;
        check(controller.calculate(invalid).turretMotorPower == 0, "T9 fault zero");
        check(!memory.getBoolean(controller), "T9 derivative memory discarded");
        TurretCommand recovered = controller.calculate(aim(-179, -150));
        check(recovered.desiredTurretAngle < 0 && recovered.requestedChassisOmega < 0,
                "T9 old branch / unwind discarded");
        near(recovered.turretMotorPower, TurretConstants.KP * recovered.turretAngleError,
                "T9 recovery uses fresh P only");
        invalid = aim(0, 0); invalid.poseValid = false; controller.calculate(invalid);
        check(!memory.getBoolean(controller), "pose fault resets derivative");
        for (double bad : new double[] {0, -.02, 1e-10, 1, Double.NaN,
                Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            controller.calculate(aim(179, 150));
            TurretState s = aim(90, 0); s.dt = bad;
            TurretCommand c = controller.calculate(s);
            check(!c.tracking && c.turretMotorPower == 0 && c.requestedChassisOmega == 0, "T10 controller dt");
            check(!memory.getBoolean(controller), "T10 resets memory");
            near(gate().apply(.1, 0, true, bad, true, true, false), 0, "T10 hardware dt");
            TurretEncoderSample sample = valid(0);
            sample.update(reading(1), bad, 1, 0, true, 10);
            check(!sample.valid && Double.isNaN(sample.angleRad), "T10 sensor dt");
        }
        // T7/T8: original rear branch + both-side recovery simulation execute in SafetyRegressionScenarios.
    }
    private static void operator() {
        TurretBenchSession session = new TurretBenchSession();
        TurretBenchSession.Readiness ready = new TurretBenchSession.Readiness();
        TurretBenchSession.Input in = new TurretBenchSession.Input();
        TurretEncoderSample sample = valid(0);
        in.arm = in.deadman = in.right = true;
        near(session.update(in, ready, sample, .02), 0, "sensor-only despite buttons");
        check(session.mode == TurretBenchSession.Mode.SENSOR_ONLY && !session.armed, "default disarmed");
        in = new TurretBenchSession.Input(); in.nextMode = true;
        session.update(in, ready, sample, .02);
        in.nextMode = false; in.arm = true;
        session.update(in, ready, sample, .02);
        check(!session.armed, "checklist blocks sign mode");
        ready.openLoop = true;
        in.arm = false; session.update(in, ready, sample, .02);
        in.arm = true; session.update(in, ready, sample, .02);
        check(session.armed, "neutral fresh A arms");
        in.arm = false; in.deadman = in.right = true;
        check(session.update(in, ready, sample, .02) > 0, "tiny sign output enabled");
        in.deadman = false;
        near(session.update(in, ready, sample, .02), 0, "T5 session release immediate zero");
        in.deadman = true;
        session.update(in, ready, new TurretEncoderSample(), .02);
        check(!session.armed, "fault disarms");
        near(session.update(in, ready, sample, .02), 0, "valid data cannot automatically rearm");
        in.stop = true; session.update(in, ready, sample, .02);
        check(session.emergencyStop && !session.armed && session.mode == TurretBenchSession.Mode.SENSOR_ONLY,
                "estop locks out powered mode");
        in.stop = false; in.reset = true; session.update(in, ready, sample, .02);
        check(session.emergencyStop, "reset rejected with deadman held");
        in.reset = false; in.deadman = in.right = false; session.update(in, ready, sample, .02);
        in.reset = true; session.update(in, ready, sample, .02);
        check(!session.emergencyStop && !session.armed, "neutral reset stays unarmed sensor mode");
        TurretBenchSession overlap = new TurretBenchSession();
        TurretBenchSession.Input chord = new TurretBenchSession.Input();
        chord.reset = chord.arm = chord.nextMode = true;
        overlap.update(chord, ready, sample, .02);
        check(overlap.mode == TurretBenchSession.Mode.SENSOR_ONLY && !overlap.armed,
                "reset cannot simultaneously select and arm a powered mode");
        chord.reset = chord.arm = chord.nextMode = false; overlap.update(chord, ready, sample, .02);
        chord.arm = chord.nextMode = true; overlap.update(chord, ready, sample, .02);
        check(overlap.mode == TurretBenchSession.Mode.SIGN_TEST && !overlap.armed,
                "mode change requires subsequent separate arm edge");
        session.mode = TurretBenchSession.Mode.SMALL_STEPS;
        in = new TurretBenchSession.Input(); in.arm = true;
        session.update(in, ready, sample, .02); check(!session.armed, "closed loop needs extra gates");
        ready.closedLoop = true; in.arm = false; session.update(in, ready, sample, .02);
        in.arm = true; session.update(in, ready, sample, .02);
        in.arm = false; in.deadman = in.up = true;
        check(session.update(in, ready, sample, .02) > 0, "small P step");
        near(session.targetRad, Math.toRadians(20), "small step upper bound");
        for (int i = 0; i < 20; i++) {
            in.right = false; session.update(in, ready, sample, .02);
            in.right = true; session.update(in, ready, sample, .02);
        }
        near(session.targetRad, Math.toRadians(20), "steps cannot exceed small range");
        session.mode = TurretBenchSession.Mode.SWEEP; session.update(in, ready, sample, .02);
        check(!session.armed, "sweep blocked until small tests passed");
        ready.sweep = true; in = new TurretBenchSession.Input(); in.arm = true;
        session.update(in, ready, sample, .02);
        in.arm = false; in.deadman = in.up = true;
        session.update(in, ready, sample, .02);
        near(session.targetRad, TurretHardwareConfig.BENCH_TEST_MAX_RAD, "sweep conservative max");
        in.up = false; in.down = true; session.update(in, ready, sample, .02);
        near(session.targetRad, TurretHardwareConfig.BENCH_TEST_MIN_RAD, "sweep conservative min");
    }
    private static void stopping() {
        TurretStoppingMeasurement m = new TurretStoppingMeasurement();
        m.update(.03, valid(10), 1);
        check(m.update(0, valid(12), 2, .02), "zero event captured");
        near(m.zeroSampleAgeSec, .02, "zero sample age recorded");
        near(m.zeroAngleRad, Math.toRadians(12), "zero angle");
        m.update(0, valid(15), 3); m.update(0, valid(14), 4);
        near(m.distanceRad, Math.toRadians(3), "extreme after zero preserved");
        m.update(0, new TurretEncoderSample(), 5);
        check(m.incomplete && !m.active, "fault marks stopping trace incomplete");
        m.update(-.03, valid(-10), 6); m.update(0, valid(-12), 7);
        m.update(0, valid(-16), 8);
        near(m.distanceRad, Math.toRadians(4), "negative stopping distance");
    }
    private static void near(double actual, double expected, String label) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-9, label + ": " + actual);
    }
    private static void check(boolean value, String label) {
        checks++; if (!value) throw new AssertionError(label);
    }
}
