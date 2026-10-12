# Turret bench readiness — October 2, 2026

**Updated October 12, 2026:** the Axon CR adapter no longer throws when `CR_CALIBRATION` is null, so SENSOR_ONLY can commission the analog absolute sensor with PWM disabled. A CR_DEADBAND bench step measures neutral and pulse deadband. Stop/E-stop write the calibrated neutral then disable PWM; a CR servo has no brake and can backdrive. Analog volts now reject rails, an unconfigured valid band, and optional stuck samples. Closed-loop and the bench IO latch a wrong-direction/stall trip. Sign-test power is derived from the measured deadband and refuses if it cannot clear it.

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

Analog conversion now rejects ADC rails (≤0 V or ≥ `getMaxVoltage()`), requires a configured valid band inside those rails (`ANALOG_VALID_MIN_VOLTS` / `ANALOG_VALID_MAX_VOLTS`, default NaN), and optionally trips a stuck/zero-noise window. A near-0 V zero is no longer treated as a live sensor by itself. Rate plausibility remains. Physical unplug, hand-turn, and independent-angle checks are still required before enabling motion. Software cannot infer correct wiring from a plausible in-band voltage. The same rail/band idea is applied to launcher pitch and gap analog inputs.

## Motor and safety

Yaw is an Axon CR servo (`turretYaw`), not a DC motor: there is no `RUN_WITHOUT_ENCODER` / `BRAKE` mode. `MOTOR_SIGN` is the CR calibration sign. SDK `FORWARD` is used; positive normalized demand must move CCW. `ENCODER_SIGN` independently normalizes sensing. Direction signs accept only ±1. A latched wrong-direction/stall trip zeros output when commanded motion and measured velocity disagree.

I/O maps the CR servo when calibration exists (sensor-only may omit it), writes calibrated neutral then disables PWM during init/stop, stops on invalid readings, checks sample freshness, enforces continuous passed arm/deadman/E-stop inputs, clips to the bench cap, and blocks outward power at the intersection of bench, operating, and physical limits. Inward motion at or beyond bench/operating limits remains allowed, provided the reading remains inside confirmed physical travel. Outside physical travel, all motion is inhibited as an impossible measurement. Generic `TurretIO.setMotorPower()` cannot arm this hardware implementation; the bench calls the explicitly gated `command()` method.

Placeholder bench range ±90 degrees, cap 0.15, requested sign-test power 0.03, P test gain 0.15 power/rad, KD=KV=0. Sign-test power is raised to just outside the measured CR deadband when that deadband exists, and is refused if it still sits inside the deadband or exceeds the bench cap. These are **bench-only unconfirmed values**, not measured recommendations or tuned gains. All confirmation flags default false; zero, analog transfer/valid band, and CR calibration are NaN/null. Closed loop defaults disabled. Smaller cable-safe ranges must be used if the rig requires them.

Stop and E-stop write the calibrated Axon CR neutral pulse (never SDK zero as a substitute) and then disable PWM on that port (`PwmControl.setPwmDisable`, with `ServoControllerEx.setServoPwmDisable` as fallback). Uncalibrated CR never writes SDK zero. **Backdrive caveat:** a CR servo has no brake; PWM disable lets the ring be moved by gravity, belts, or contact. Keep mechanical stops and operator access to power removal.

Deadman release/B stop commands zero before acquisition or telemetry work. Mode changes disarm. Data/timing faults disarm and clear measurement/controller history. E-stop returns to sensor-only and stays latched until the neutral reset chord; it does not auto-arm. Motor write failures remain latched; if zero cannot be written, telemetry reports unknown actual output and instructs physical stop. Use Driver Station STOP/disconnect battery for that condition.

This is a synchronous OpMode: it cannot provide a hard real-time stop guarantee during an OS stall, disconnected Hub, or a failed write. No worker/watchdog architecture was added. Physical stops and operator access to power removal remain necessary.

## Driver Station OpMode and controls

Select **Turret Hardware Bench** in group **Test**. INIT and START both default to SENSOR_ONLY, with motor zero. No pose, drivetrain, camera, field planner, autonomous, or homing is involved.

| Control | Action |
| --- | --- |
| LB rising edge, RB/DPAD released | Cycle SENSOR_ONLY → CR_DEADBAND → SIGN_TEST → SMALL_STEPS → SWEEP; disarms and resets target to 0. |
| A rising edge, RB/DPAD released | Arm selected powered mode if its checklist and sensor/timing are valid. |
| Hold RB | Continuous motor enable in every powered mode; release commands zero immediately in the next OpMode cycle. SENSOR_ONLY cannot command the CR servo. |
| CR_DEADBAND: RB + DPAD right/left | Nudge the measurement pulse; telemetry shows turret rate and a paste line for `CR_CALIBRATION`. |
| SIGN_TEST: RB + DPAD right/left | ± derived sign-test power (outside measured CR deadband); opposing inputs produce zero. |
| SMALL_STEPS: DPAD right/left rising edges | Target ±10-degree increments, bounded to ±20 degrees and bench limits. |
| SMALL_STEPS: DPAD up/down; X | Target +20/-20/0 degrees. Hold RB to actuate. |
| SWEEP: DPAD right/left; up/down; X | 10-degree increments; positive/negative bench endpoint; zero. Hold RB to actuate. No automatic motion sequence. |
| B | Immediate zero command, latched E-stop, sensor-only mode. |
| BACK+START rising edge, RB/DPAD released | Clear E-stop into disarmed sensor-only; reselect mode and press A deliberately. |

Open-loop gates: valid absolute angle/velocity, calibrated zero, verified output-side 1:1 sensing/wiring/transfer, encoder sign verified by hand, measured limits and bench power confirmed. Motor sign is established using SIGN_TEST. Closed-loop additionally requires motor sign verified and explicit CLOSED_LOOP_ENABLED. SWEEP additionally requires SMALL_STEPS_PASSED. Runtime readiness flags are not gamepad toggles: manually update configuration only after performing the physical checks.

Telemetry shows mode/arming/E-stop, backend/fault/inhibit, raw units and validity, signed unoffset/calibration suggestion/configured zero, actual rad/deg and velocity, target and zero target velocity, error, controller demand/final output, directional limits, physical/operating/bench intervals/cap, loop dt, battery voltage, and stopping fields. Actual values remain NaN when unavailable. Unwind active=false, branch=NONE, chassis omega=0 are explicitly labeled direct bench behavior. The field recovery state machine is tested by existing desktop regressions; this deliberately conservative direct bench mode does not drive physical rear-sector recovery.

Robot Controller log tag `TurretBench` emits diagnostics at 10 Hz and at command-zero events. It records monotonic time, mode, arm/validity, raw value, angle/velocity/target/demand/applied output, dt, faults/inhibit, zero command time/angle, extreme angle, sample age and stop distance. No automatic fitting or tuning is performed.

Stopping measurements use the most recent angle at the zero command and track the greatest subsequent excursion in the direction of motion. For immediate deadman/E-stop shutdown this angle is from the preceding acquisition: **sample age is logged explicitly**, so this is a sampled measurement with uncertainty, not an exact continuous-time stopping distance. Invalid data marks a trace incomplete. Starting movement again ends the previous trace; the next stop starts a new one. Do not manually move the ring during a stopping trace. Stop/E-stop write calibrated CR neutral then disable PWM; a CR servo has no brake, so backdrive is possible. Recharacterize stopping distance on this actuator; do not treat DC-motor BRAKE data as applicable.

## Verification

Final desktop results: **1,031 checks passed**: planner 99, tracker 180, history 23, turret safety 72, Pedro mapping 45, existing safety regressions 498, new turret bench 114. All 917 existing assertions passed unchanged. Rear 179° → 181° remains +160° → +160° with positive ramped chassis request; both unwind simulations clear in 0.68 s. `git diff --check` passed. Android `:TeamCode:compileDebugJavaWithJavac` and `:TeamCode:assembleDebug` succeeded offline using the installed Android Studio JDK and existing Gradle cache; artifact `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`. Warnings concern Java 8/source compatibility and pre-existing deprecated SDK/Gradle APIs; no build failures remain.

T1 invalid sensor/output=0; T2/T3 directional blocking with inward recovery; T4 power cap; T5 deadman; T6 encoder sign; T7 rear-sector continuity and T8 unwind completion in unchanged existing regressions; T9 invalid→valid baseline/controller state reset; T10 zero/negative/tiny/large/nonfinite dt. Additional checks cover startup angles at -120/-45/+60/+140/+80/+10 degrees, raw wrap velocity, missing calibration/1:1 sensing, impossible rate, E-stop/reset/rearm/mode gates (including simultaneous-button rejection), step/sweep bounds, and both stopping directions/sample age.

Desktop checks execute the real pure safety/operator/conversion classes and existing Pedro follower simulations. They do not emulate electrical wiring, SDK transactions, motor inertia, or gamepad/Driver Station latency. The Android build verifies SDK integration. The existing SDK simulated OpMode builds but was not run on a Driver Station.

## Ordered physical procedure (blocked until wiring is resolved)

1. **Sensor only.** Keep the CR servo PWM disabled (the bench starts that way and SENSOR_ONLY cannot command output). Select `BACKEND=ANALOG_ABSOLUTE` and map `turretEncoder` as **Analog Input**. Map `turretYaw` if the servo is already installed; it is optional until CR measurement. Hand-turn to both hard stops. Record voltage range, wrap, and the signed unoffset at mechanical forward. Fill `ANALOG_VALID_MIN_VOLTS` / `ANALOG_VALID_MAX_VOLTS` inside the observed travel and off the rails, then transfer endpoints and zero. Confirm CCW hand-turn increases angle before `ENCODER_SIGN_VERIFIED`. STOP for rail/disconnect/stuck/out-of-band readings. Unplug should fault, not look like a legal near-zero angle.
2. **CR neutral and deadband.** Set `CR_MEASURE_START_US`, min/max/frame from the Axon programmer as a **measurement window only**, then `CR_MEASURE_CLEARED` when the robot is safe. Select CR_DEADBAND, arm, hold RB, nudge the pulse with DPAD. Telemetry shows measured turret rate. Stationary pulse is the candidate neutral; first motion on each side is the deadband. Paste the printed `AxonCrCalibration(...)` into `CR_CALIBRATION`, verify stationary at operating voltage, then set `CR_MODE_AND_NEUTRAL_VERIFIED`. B / RB release write nothing but PWM disable until that calibration exists.
3. **Verify signs.** Inspect cable-safe clearance. Select SIGN_TEST. The session refuses if sign-test power cannot clear the measured deadband. Briefly hold RB+right; require angle/velocity to increase. Repeat left. A wrong `MOTOR_SIGN` now latches WRONG DIRECTION / STALL TRIP and zeros output. STOP/B at the first mismatch. Confirm `MOTOR_SIGN_VERIFIED` only after both directions.
4. **Measure limits.** With PWM disabled / servo unpowered if needed, inspect mechanical and cable-safe stops; never search for them under power. Set physical, operating, and conservative bench ranges. Recheck the sensor, set `LIMITS_CONFIRMED` and `BENCH_POWER_CONFIRMED` only for measured/cable-safe values.
5. **Small closed-loop steps.** Enable `CLOSED_LOOP_ENABLED` only after startup, both signs, limits, and power cap. Select SMALL_STEPS. Hold RB for 0, ±10, ±20 degree tests. Watch the direction trip, RB release, B latch, and analog faults. Mark `SMALL_STEPS_PASSED` only after stable physical validation.
6. **Sweep.** Only after small steps and stopping clearance, select SWEEP. Keep the range conservative. End with RB release, B (neutral + PWM disable), then DS STOP. Do not continue into vision/shooter/drivetrain integration or competition tuning.

## Values and decisions still requiring physical evidence

Absolute signal/capture wiring and transfer, output-side 1:1 sensing, encoder zero/direction, motor direction, actual physical min/max, cable-safe min/max, operating min/max, bench min/max, acceptable bench power, maximum safe turret speed, braking/coast distance versus speed/load/voltage/direction, belt tension and inertia, real kP/kD/kV. All current limits/gains/caps are placeholders. Derivative and feedforward remain disabled. A passing build is not proof of correct wiring or mechanical readiness.

## Readiness gate

Implemented: real motor TurretIO with explicit missing-signal block; no invented zero; invalid acquisition/timing → zero; sign test with tiny power; independent encoder sign; deadman; E-stop; separate conservative bench limits; directional blocking; cap; disabled-by-default closed loop and KV; diagnostic telemetry/logs; preserved rear/unwind regression coverage.

**Blocking:** selected and verified absolute backend/harness/transfer, zero and sign calibration, physical/cable-safe limits, confirmation flags and physical validation. Current defaults deliberately prevent motor output. Stop at this bench layer.
