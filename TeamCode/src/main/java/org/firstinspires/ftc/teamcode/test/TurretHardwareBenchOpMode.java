package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.turret.*;

/** Dedicated bench gate. No drive, vision, planner, homing, or automatic sweep execution. */
@TeleOp(name = "Turret Hardware Bench", group = "Test")
public final class TurretHardwareBenchOpMode extends OpMode {
    private RevTurretIO io;
    private TurretBenchSession session = new TurretBenchSession();
    private final TurretStoppingMeasurement stopping = new TurretStoppingMeasurement();
    private final TurretBenchSession.Readiness readiness = TurretBenchSession.Readiness.configured();
    private String initFault = "";
    private long lastNanos, epochNanos;
    private double dt, lastLogSec;

    @Override public void init() {
        epochNanos = lastNanos = System.nanoTime();
        telemetry.setMsTransmissionInterval(100);
        try { io = new RevTurretIO(hardwareMap); }
        catch (RuntimeException failure) { initFault = "HARDWARE INIT FAILED: " + failure.getMessage(); }
        telemetry.addLine("SENSOR ONLY DEFAULT. B = latched emergency stop.");
        telemetry.addData("Hardware", initFault.isEmpty() ? "mapped; motor zero" : initFault);
    }
    @Override public void init_loop() { cycle(false); }
    @Override public void start() {
        session = new TurretBenchSession();
        lastNanos = System.nanoTime();
        if (io != null) io.stop();
    }
    @Override public void loop() { cycle(true); }
    private void cycle(boolean started) {
        long now = System.nanoTime();
        dt = (now - lastNanos) * 1e-9;
        lastNanos = now;
        double seconds = (now - epochNanos) * 1e-9;
        if (io == null) {
            telemetry.addLine("ABSOLUTE POSITION NOT VALID — MOTOR DISABLED");
            telemetry.addData("Fault", initFault);
            return;
        }
        try {
            // Zero before acquisition/telemetry work on release or E-stop; no slow read
            // may delay the previous cycle's motor shutdown for these inputs.
            boolean immediateZeroEvent = false;
            if (!started || !gamepad1.right_bumper || gamepad1.b || !TurretEncoderSample.validDt(dt)) {
                io.stop();
                immediateZeroEvent = stopping.update(io.appliedPower, io.sample,
                        (System.nanoTime() - epochNanos) * 1e-9, io.sampleAgeSec());
            }
            if (started && gamepad1.b) {
                session.emergencyStop = true; session.armed = false;
                session.mode = TurretBenchSession.Mode.SENSOR_ONLY;
            }
            io.read(dt);
            TurretBenchSession.Input input = new TurretBenchSession.Input();
            if (started) {
                input.nextMode = gamepad1.left_bumper;
                input.arm = gamepad1.a;
                input.reset = gamepad1.back && gamepad1.start;
                input.stop = gamepad1.b;
                input.deadman = gamepad1.right_bumper;
                input.left = gamepad1.dpad_left; input.right = gamepad1.dpad_right;
                input.up = gamepad1.dpad_up; input.down = gamepad1.dpad_down;
                input.center = gamepad1.x;
            }
            double demand = session.update(input, readiness, io.sample, dt);
            if (!io.motorFault.isEmpty()) session.armed = false;
            io.command(demand, started && session.armed, input.deadman, session.emergencyStop);
            if (!io.motorFault.isEmpty()) session.armed = false;
            boolean zeroEvent = stopping.update(io.appliedPower, io.sample,
                    (System.nanoTime() - epochNanos) * 1e-9, io.sampleAgeSec());
            zeroEvent |= immediateZeroEvent;
            if (zeroEvent || seconds - lastLogSec >= .1) {
                RobotLog.ii("TurretBench", "t=%.4f mode=%s armed=%s valid=%s raw=%.5f "
                                + "angle=%.5f vel=%.5f target=%.5f demand=%.5f output=%.5f "
                                + "dt=%.5f sensorDt=%.5f fault=%s inhibit=%s zeroT=%.4f zeroAngle=%.5f extreme=%.5f "
                                + "zeroSampleAge=%.5f stopDistance=%.5f incomplete=%s",
                        seconds, session.mode, session.armed, io.sample.valid, io.sample.raw,
                        io.getAngleRad(), io.getVelocityRadPerSec(), session.targetRad,
                        session.rawPower, io.appliedPower, dt, io.acquisitionDtSec, io.sample.fault,
                        session.reason.isEmpty() ? io.safety.reason : session.reason,
                        stopping.zeroTimeSec, stopping.zeroAngleRad, stopping.extremeAngleRad,
                        stopping.zeroSampleAgeSec, stopping.distanceRad, stopping.incomplete);
                lastLogSec = seconds;
            }
            display();
        } catch (RuntimeException failure) {
            session.armed = false; session.emergencyStop = true;
            session.mode = TurretBenchSession.Mode.SENSOR_ONLY;
            io.stop();
            telemetry.addData("BENCH FAULT / MOTOR DISABLED", failure.toString());
        }
    }
    private void display() {
        telemetry.addLine(io.sample.valid ? "ABSOLUTE POSITION VALID"
                : "ABSOLUTE POSITION NOT VALID — MOTOR DISABLED");
        telemetry.addData("Mode / armed / E-stop", "%s / %s / %s",
                session.mode, session.armed ? "YES" : "NO", session.emergencyStop);
        telemetry.addData("Fault", "%s %s", io.sample.fault, io.motorFault);
        telemetry.addData("OUTPUT STATUS", session.reason.isEmpty()
                ? (io.safety.reason.isEmpty() ? "ENABLED" : io.safety.reason) : "OUTPUT BLOCKED: " + session.reason);
        telemetry.addData("Backend", TurretHardwareConfig.BACKEND);
        telemetry.addData("Raw / valid", "%.6f %s / %s", io.sample.raw, io.sample.units, io.sample.valid);
        telemetry.addData("Unoffset / required zero at forward rad", io.sample.unoffsetRad);
        telemetry.addData("Configured zero rad", TurretHardwareConfig.ENCODER_ZERO_OFFSET_RAD);
        telemetry.addData("Actual angle deg / rad", "%.3f / %.6f", Math.toDegrees(io.getAngleRad()), io.getAngleRad());
        telemetry.addData("Actual velocity deg/s", Math.toDegrees(io.getVelocityRadPerSec()));
        telemetry.addData("Target deg / velocity rad/s", "%.2f / 0", Math.toDegrees(session.targetRad));
        telemetry.addData("Error deg", Math.toDegrees(session.errorRad));
        telemetry.addData("Raw / post-safety output", "%.5f / %.5f", session.rawPower, io.appliedPower);
        telemetry.addData("Positive / negative limit active", "%s / %s", io.safety.positiveLimit, io.safety.negativeLimit);
        telemetry.addData("Physical deg (PLACEHOLDER)", "%.1f .. %.1f",
                Math.toDegrees(TurretConstants.PHYSICAL_MIN_RAD), Math.toDegrees(TurretConstants.PHYSICAL_MAX_RAD));
        telemetry.addData("Operating deg (PLACEHOLDER)", "%.1f .. %.1f",
                Math.toDegrees(TurretConstants.OPERATING_MIN_RAD), Math.toDegrees(TurretConstants.OPERATING_MAX_RAD));
        telemetry.addData("Bench deg / cap (UNCONFIRMED)", "%.1f .. %.1f / %.3f",
                Math.toDegrees(io.safety.minRad), Math.toDegrees(io.safety.maxRad), io.safety.cap);
        telemetry.addData("Checklist open / closed / sweep", "%s / %s / %s",
                readiness.openLoop, readiness.closedLoop, readiness.sweep);
        telemetry.addLine("Unwind active=false; branch=NONE (direct bench target); chassis omega=0");
        telemetry.addData("Loop dt s", dt);
        telemetry.addData("Acquisition dt s", io.acquisitionDtSec);
        double battery = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 0 && Double.isFinite(volts)) battery = Double.isNaN(battery) ? volts : Math.min(battery, volts);
        }
        telemetry.addData("Battery V", battery);
        telemetry.addData("Zero command t / angle deg / velocity deg/s", "%.4f / %.3f / %.3f",
                stopping.zeroTimeSec, Math.toDegrees(stopping.zeroAngleRad), Math.toDegrees(stopping.zeroVelocityRadPerSec));
        telemetry.addData("Post-zero extreme / distance deg / incomplete", "%.3f / %.3f / %s",
                Math.toDegrees(stopping.extremeAngleRad), Math.toDegrees(stopping.distanceRad), stopping.incomplete);
        telemetry.addData("Zero angle sample age s (measurement uncertainty)", stopping.zeroSampleAgeSec);
        telemetry.addLine("LB next mode (RB released); A arm (RB/DPAD released); hold RB to power");
        telemetry.addLine("DPAD L/R sign or 10° step; U/D ±20° or bench endpoint; X target 0°");
        telemetry.addLine("B E-STOP; release RB/DPAD, BACK+START reset to SENSOR ONLY, then reselect + A");
    }
    @Override public void stop() { if (io != null) io.stop(); }
}
