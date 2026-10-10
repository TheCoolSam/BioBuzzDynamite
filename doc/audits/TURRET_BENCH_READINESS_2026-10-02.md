# Turret bench readiness — October 2, 2026

**NOT READY FOR POWERED TURRET TEST**

The hardware/test layer is implemented, but the team explicitly selected “Undecided — keep acquisition unselected and motor disabled.” Sensor-only physical readiness is also blocked until an absolute signal path is decided and wired. Running the new OpMode with the current configuration produces `ABSOLUTE ACQUISITION UNSELECTED` and zero output, not a fabricated angle. No hardware was connected, moved, calibrated, or tuned in this task.

## Inspection before editing

- Branch `main`; HEAD `7ffbe3fadaaccb56ff23b525fce57e0da66fe601`.
- Initial status: four existing staged additions: `doc/audits/BIOBUZZ_AUDIT_2026-10-01.md`, `doc/audits/reproductions/AuditProbe.java`, `OBSERVED_OUTPUT.txt`, and `PedroAuditProbe.java`. Preserved; no commit or staging performed.
- Original turret files: `TurretCommand.java`, `TurretConstants.java`, `TurretController.java`, `TurretIO.java`, `TurretState.java`, `TurretStateAdapter.java` in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/turret/`.
- Original tests: `test/TurretSafetyScenarios.java` (72 assertions), turret groups in `test/SafetyRegressionScenarios.java`, and SDK-based simulated `test/TurretControllerTestOpMode.java`. Six desktop suites existed; no real TurretIO existed. Only the private simulated implementation in the test OpMode implemented the interface.
- Original TurretIO: `getAngleRad()`, `getVelocityRadPerSec()`, `setMotorPower(double)`. Angles radians, chassis-relative CCW positive; normalized positive power must move CCW. Now also provides `stop()`; the existing simulation remains compatible.
- Original constants, all placeholders: physical -165/+165 degrees; operating -160/+160; unwind enter/exit 140/130; unwind KP 3; requested chassis omega cap 2 rad/s; omega ramp 4 rad/s²; turret KP 2, KD 0, KV 0; output range -1/+1.
- Invalid pose or nonfinite turret/target/state data returned a zero-power, zero-chassis-request hold. The adapter preserved invalid measurements, rather than replacing them with zero. Hold command metadata could use 0 when there was no finite measurement; it was not an acquired angle.
- Original normal output: P + derivative of error + desired-rate feedforward; finite check, ±1 clamp, outward block at physical limits only. Desired position clamped to operating and physical intervals; position error deliberately unwrapped to avoid rear-sector travel. Recovery already latched a side across 179/181 and completed unwind.
- Hazards found: fault holds retained derivative and branch/unwind state; finite zero/negative/huge dt still permitted P output; no independent bench output gate, real acquisition, zero calibration, startup proof, deadman, E-stop, power cap, or hardware diagnostic OpMode. Physical stop placeholders alone were inadequate for first power.
- KD and KV are zero, so differentiated target switches and infeasible feedforward do not currently drive hardware. They remain future tuning/design concerns, not enabled bench features. No trajectory/controller redesign was performed.

## Changed files

Paths below are relative to the repository. Java paths share `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

| File | Change |
| --- | --- |
| `turret/TurretController.java` | Reset all dynamic state after invalid pose/data/timing; reject dt outside 0.0001–0.2 s. |
| `turret/TurretState.java` | Document timing contract. |
| `turret/TurretIO.java` | Document real implementation; add compatible default stop. |
| `turret/TurretHardwareConfig.java` | Central names, acquisition selection, sign/zero, readiness flags, bench limits/caps/gains. |
| `turret/AbsoluteTurretSensor.java` | Backend seam supplying raw units, absolute turns, and explicit faults. |
| `turret/TurretEncoderSample.java` | Pure absolute conversion, calibration diagnostics, wrap-safe velocity, plausibility/timing validation. |
| `turret/TurretOutputSafety.java` | Independent final cap, directional limit, deadman, arm, fault, and timing gate. |
| `turret/RevTurretIO.java` | Real REV motor adapter and selectable absolute acquisition; independent guarded output. |
| `turret/TurretBenchSession.java` | Pure progressive bench modes and deliberate arming/E-stop state machine. |
| `turret/TurretStoppingMeasurement.java` | Observe command-zero angle, time/sample age, velocity, maximum subsequent excursion. |
| `test/TurretHardwareBenchOpMode.java` | Dedicated Driver Station hardware bench control and telemetry/logging. |
| `test/TurretBenchScenarios.java` | Hardware-independent safety, acquisition, operator, and stopping regressions. |
| `tools/run-desktop-tests.ps1` | Include new suite; omit SDK-dependent RevTurretIO from desktop javac. Android build compiles it. |
| `doc/audits/TURRET_BENCH_READINESS_2026-10-02.md` | This audit and procedure. |

## Absolute encoder decision and implementation

The current [AndyMark am-5200 non-CAN specifications](https://andymark.com/products/hex-bore-encoder?variant=45166126104748) list quadrature A/B, absolute PWM, and absolute analog. The public product page does not establish this team's selected output, connector wiring, or analog voltage-to-angle transfer. Neither does this repo. Quadrature alone cannot establish arbitrary startup angle.

`BACKEND=UNSELECTED` is the actual selected backend. No sensor API or wiring scheme is active. PWM selection remains explicitly unavailable until a supported capture adapter is implemented; reading `DigitalChannel.getState()` does not measure pulse width. A future PWM backend must supply verified absolute turns through `AbsoluteTurretSensor`; it must not use incremental counts as startup position.

A conditional analog adapter exists for a future **confirmed linear analog path**, using FTC `AnalogInput.getVoltage()` (raw volts). It is gated by `WIRING_AND_TRANSFER_CONFIRMED`, finite configured transfer endpoints, and the input API's measurable range. Formula: `turns=(rawVoltage-zeroTurnVoltage)/(fullTurnVoltage-zeroTurnVoltage)`, with `0 <= turns < 1`. The endpoints intentionally remain NaN; neither 3.3 V nor 5 V is assumed. `getMaxVoltage()` is used only to reject an incompatible range, never as the encoder transfer scale. See [REV Analog input configuration](https://docs.revrobotics.com/duo-control/sensors/analog) and [FTC AnalogInput API](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.3.0/com/qualcomm/robotcore/hardware/AnalogInput.html).

Required physical information: exact non-CAN device/revision, selected analog/PWM signal and device mode, verified connector pin numbers/functions and cable continuity, supply and output voltage, ground reference, Hub channel or capture interface, absolute transfer function, and confirmation that the sensor is output-side 1:1 over less than a full revolution. If its actual transfer is not linear, replace the conditional analog adapter; do not fill in endpoints to approximate an undocumented function.

Conversion is `signedUnoffset = ENCODER_SIGN * turns * 2π`, `angle=wrap(signedUnoffset-ENCODER_ZERO_OFFSET_RAD)` into [-π,π). The mechanical forward datum defines zero. At forward, manually record the displayed signed unoffset radians as the required zero offset and place it in configuration. No constant or persistent state is automatically rewritten. An explicit 1:1 confirmation and finite zero are required; NaN never becomes angle 0.

First valid acquisition supplies absolute angle immediately, with no motion/homing. It requires a second valid sample before declaring full angle/velocity health and permitting power. Velocity is wrapped angle difference over the monotonic acquisition interval, not incremental motor counts or assumed loop timing; a raw wrap near 360/0 cannot create a 360-degree velocity spike. Both acquisition timing and loop timing are validated and displayed. Invalid readings/timing/rate discard the baseline; recovery requires a fresh baseline and rearming. Real I/O also rejects positions beyond the physical interval, so a rear ±π jump cannot be treated as legal recovery.

Finite voltages alone cannot distinguish all disconnected, stuck, or miswired analog sensors from legitimate positions. Physical independent-angle, unplug, and signal-health validation is still required before enabling motion; the final sensing strategy may need additional health detection. Software cannot infer correct wiring from a plausible voltage.

## Motor and safety

`DcMotor` uses `RUN_WITHOUT_ENCODER`, SDK `FORWARD`, and `BRAKE`; the internal motor encoder is not treated as absolute turret position. `MOTOR_SIGN` multiplies normalized final power once in I/O. Positive normalized output must move CCW; `ENCODER_SIGN` independently normalizes sensing. Direction signs accept only ±1.

I/O maps and zeros the motor during init, stops on invalid readings, checks sample freshness, enforces continuous passed arm/deadman/E-stop inputs, clips to the bench cap, and blocks outward power at the intersection of bench, operating, and physical limits. Inward motion at or beyond bench/operating limits remains allowed, provided the reading remains inside confirmed physical travel. Outside physical travel, all motion is inhibited as an impossible measurement. Generic `TurretIO.setMotorPower()` cannot arm this hardware implementation; the bench calls the explicitly gated `command()` method.

Placeholder bench range ±90 degrees, cap 0.15, sign-test power 0.03, P test gain 0.15 power/rad, KD=KV=0. These are **bench-only unconfirmed values**, not measured recommendations or tuned gains. All confirmation flags default false; zero and analog transfer are NaN. Closed loop defaults disabled. Smaller cable-safe ranges must be used if the rig requires them.

Deadman release/B stop commands zero before acquisition or telemetry work. Mode changes disarm. Data/timing faults disarm and clear measurement/controller history. E-stop returns to sensor-only and stays latched until the neutral reset chord; it does not auto-arm. Motor write failures remain latched; if zero cannot be written, telemetry reports unknown actual output and instructs physical stop. Use Driver Station STOP/disconnect battery for that condition.

This is a synchronous OpMode: it cannot provide a hard real-time stop guarantee during an OS stall, disconnected Hub, or a failed write. No worker/watchdog architecture was added. Physical stops and operator access to power removal remain necessary.

## Driver Station OpMode and controls

Select **Turret Hardware Bench** in group **Test**. INIT and START both default to SENSOR_ONLY, with motor zero. No pose, drivetrain, camera, field planner, autonomous, or homing is involved.

| Control | Action |
| --- | --- |
| LB rising edge, RB/DPAD released | Cycle SENSOR_ONLY → SIGN_TEST → SMALL_STEPS → SWEEP; disarms and resets target to 0. |
| A rising edge, RB/DPAD released | Arm selected powered mode if its checklist and sensor/timing are valid. |
| Hold RB | Continuous motor enable in every powered mode; release commands zero immediately in the next OpMode cycle. |
| SIGN_TEST: RB + DPAD right/left | +0.03 / -0.03 normalized power; opposing inputs produce zero. |
| SMALL_STEPS: DPAD right/left rising edges | Target ±10-degree increments, bounded to ±20 degrees and bench limits. |
| SMALL_STEPS: DPAD up/down; X | Target +20/-20/0 degrees. Hold RB to actuate. |
| SWEEP: DPAD right/left; up/down; X | 10-degree increments; positive/negative bench endpoint; zero. Hold RB to actuate. No automatic motion sequence. |
| B | Immediate zero command, latched E-stop, sensor-only mode. |
| BACK+START rising edge, RB/DPAD released | Clear E-stop into disarmed sensor-only; reselect mode and press A deliberately. |

Open-loop gates: valid absolute angle/velocity, calibrated zero, verified output-side 1:1 sensing/wiring/transfer, encoder sign verified by hand, measured limits and bench power confirmed. Motor sign is established using SIGN_TEST. Closed-loop additionally requires motor sign verified and explicit CLOSED_LOOP_ENABLED. SWEEP additionally requires SMALL_STEPS_PASSED. Runtime readiness flags are not gamepad toggles: manually update configuration only after performing the physical checks.

Telemetry shows mode/arming/E-stop, backend/fault/inhibit, raw units and validity, signed unoffset/calibration suggestion/configured zero, actual rad/deg and velocity, target and zero target velocity, error, controller demand/final output, directional limits, physical/operating/bench intervals/cap, loop dt, battery voltage, and stopping fields. Actual values remain NaN when unavailable. Unwind active=false, branch=NONE, chassis omega=0 are explicitly labeled direct bench behavior. The field recovery state machine is tested by existing desktop regressions; this deliberately conservative direct bench mode does not drive physical rear-sector recovery.

Robot Controller log tag `TurretBench` emits diagnostics at 10 Hz and at command-zero events. It records monotonic time, mode, arm/validity, raw value, angle/velocity/target/demand/applied output, dt, faults/inhibit, zero command time/angle, extreme angle, sample age and stop distance. No automatic fitting or tuning is performed.

Stopping measurements use the most recent angle at the zero command and track the greatest subsequent excursion in the direction of motion. For immediate deadman/E-stop shutdown this angle is from the preceding acquisition: **sample age is logged explicitly**, so this is a sampled measurement with uncertainty, not an exact continuous-time stopping distance. Invalid data marks a trace incomplete. Starting movement again ends the previous trace; the next stop starts a new one. Do not manually move the ring during a stopping trace. BRAKE is the characterized zero-power behavior here; FLOAT comparisons require a deliberate separate test/configuration change.

## Verification

Final desktop results: **1,031 checks passed**: planner 99, tracker 180, history 23, turret safety 72, Pedro mapping 45, existing safety regressions 498, new turret bench 114. All 917 existing assertions passed unchanged. Rear 179° → 181° remains +160° → +160° with positive ramped chassis request; both unwind simulations clear in 0.68 s. `git diff --check` passed. Android `:TeamCode:compileDebugJavaWithJavac` and `:TeamCode:assembleDebug` succeeded offline using the installed Android Studio JDK and existing Gradle cache; artifact `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`. Warnings concern Java 8/source compatibility and pre-existing deprecated SDK/Gradle APIs; no build failures remain.

T1 invalid sensor/output=0; T2/T3 directional blocking with inward recovery; T4 power cap; T5 deadman; T6 encoder sign; T7 rear-sector continuity and T8 unwind completion in unchanged existing regressions; T9 invalid→valid baseline/controller state reset; T10 zero/negative/tiny/large/nonfinite dt. Additional checks cover startup angles at -120/-45/+60/+140/+80/+10 degrees, raw wrap velocity, missing calibration/1:1 sensing, impossible rate, E-stop/reset/rearm/mode gates (including simultaneous-button rejection), step/sweep bounds, and both stopping directions/sample age.

Desktop checks execute the real pure safety/operator/conversion classes and existing Pedro follower simulations. They do not emulate electrical wiring, SDK transactions, motor inertia, or gamepad/Driver Station latency. The Android build verifies SDK integration. The existing SDK simulated OpMode builds but was not run on a Driver Station.

## Ordered physical procedure (blocked until wiring is resolved)

1. **Sensor only.** Keep the motor power cable disconnected while establishing wiring, configured raw transfer and sensor API. Map `turretMotor` as the actual DC motor on its selected Hub motor port; only if analog is selected, map `turretEncoder` as **Analog Input** on the verified channel. PWM needs a supported capture implementation and its own configuration; do not configure it as a guessed analog input. Verify cable pinout and electrical levels independently. Manually rotate within mechanically inspected cable-safe travel; compare raw output and independent angle. At forward record zero, configure it, rebuild and reboot. STOP for missing/stuck/reversed/discontinuous/implausible readings, wrong device mode or clipping. Do not flip confirmation flags to hide faults.
2. **Absolute reboot position.** With motor disconnected and SENSOR_ONLY selected, power cycle at -120, -45, +60, +140 degrees **only where those positions are physically legal and cable-safe**; otherwise choose equivalent safe test positions spanning the measured travel. Read the same correct chassis-relative angle without movement. The placeholder ±90 powered bench interval does not prohibit sensor-only reads at these legal physical angles. STOP for any homing requirement, fabricated zero, wrong sign/offset/branch, or nonrepeatable angle. Confirm startup and manual encoder direction independently before setting ENCODER_SIGN_VERIFIED.
3. **Verify signs.** First verify hand CCW means increasing angle. Inspect the rig's mechanical/cable clearance and choose an initial safe inner interval before any motor connection. Confirm limits/cap flags only after this inspection and independent angle check. With belt/rig secured, connect motor, select SIGN_TEST, release RB/DPAD and press A. Briefly hold RB+right, release; require angle/velocity to increase. Repeat left; require decreases. STOP/B at the first mismatch, binding, cable pull, or unexpected motion. Fix signs only in configuration; repeat the test before confirming MOTOR_SIGN_VERIFIED.
4. **Tiny open loop.** Repeat short 0.03-power pulses well inside the measured interval, with RB continuously held. Verify release, B latch, neutral reset, DS STOP, sensor-fault behavior, and directional limits without approaching a mechanical stop under power. STOP for delayed shutdown, failed zero write, sensor fault, any unintended arm/motion, or excessive speed. Re-arm deliberately after faults. No disconnected-sensor test should be attempted with a hazardous mechanism energized.
5. **Measure limits.** With motor power disconnected, inspect/measure actual mechanical stop and cable-safe minima/maxima; never search for them using motor power. Set physical, operating, and conservative bench ranges in their respective constants. Leave stopping margin ample until measured. Recheck the sensor against independent angles, set LIMITS_CONFIRMED only for measured/cable-safe configuration, rebuild, and repeat sensor/sign tests. The preliminary inspected inner interval in step 3 is necessary before the first pulse; this step establishes measured full travel.
6. **Small closed-loop steps.** Only after startup, both signs, limits, belt tension/rig integrity and power cap are validated: deliberately enable CLOSED_LOOP_ENABLED. Select SMALL_STEPS and arm neutral. Hold RB for 0, +10, -10, +20, -20 degree tests within your actual bench interval. KD=KV remain 0. Monitor actual/error/raw/final power; release RB/B for oscillation, wrong-way motion, overshoot threatening clearance, invalid data or binding. Do not tune from simulation. Mark SMALL_STEPS_PASSED only after stable physical validation.
7. **Stopping distance.** In SIGN_TEST, start well inside the measured interval, apply a controlled small pulse and release RB. Record zero command angle/time/sample age/velocity, extreme angle and distance from telemetry/logs; allow ring to settle without manual movement. Repeat both directions. Reject incomplete traces and account for sampling uncertainty; do not derive a final safety margin from one trace. No higher power/speed is automatically enabled. STOP if stopping travel threatens the inner clearance or is inconsistent.
8. **Larger sweeps.** Only after small steps and stopping clearance succeed, select SWEEP. Arm neutral, hold RB and request alternating bench endpoints or intermediate 10-degree steps. Watch stopping behavior, cable motion, repeatability and measured limits. Keep the range conservative; no field rear-branch target or drivetrain request is executed. STOP for cable stress, repeatability drift, limit violation, increasing oscillation/speed, sensor fault, or motor fault. End with RB release, B, then DS STOP. Do not continue into vision/shooter/drivetrain integration or competition tuning.

## Values and decisions still requiring physical evidence

Absolute signal/capture wiring and transfer, output-side 1:1 sensing, encoder zero/direction, motor direction, actual physical min/max, cable-safe min/max, operating min/max, bench min/max, acceptable bench power, maximum safe turret speed, braking/coast distance versus speed/load/voltage/direction, belt tension and inertia, real kP/kD/kV. All current limits/gains/caps are placeholders. Derivative and feedforward remain disabled. A passing build is not proof of correct wiring or mechanical readiness.

## Readiness gate

Implemented: real motor TurretIO with explicit missing-signal block; no invented zero; invalid acquisition/timing → zero; sign test with tiny power; independent encoder sign; deadman; E-stop; separate conservative bench limits; directional blocking; cap; disabled-by-default closed loop and KV; diagnostic telemetry/logs; preserved rear/unwind regression coverage.

**Blocking:** selected and verified absolute backend/harness/transfer, zero and sign calibration, physical/cable-safe limits, confirmation flags and physical validation. Current defaults deliberately prevent motor output. Stop at this bench layer.
