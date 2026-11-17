# Tuning quickstart: shooter flywheel and Limelight auto-aim

Two OpModes, both in the "Bench" group on the Driver Station. Both put every tunable in
FTC Dashboard and show on the DS screen whether Dashboard edits are really arriving.

## 0. Connect Dashboard (do this first, every session)

1. Laptop on the Control Hub Wi-Fi (`FTC-xxxx`).
2. Browser: `http://192.168.43.1:8080/dash`
3. INIT the OpMode. In the **Configuration** tab a new group appears:
   `Shooter Bench` or `Auto Aim`. It disappears when the OpMode stops.
4. **Test the link before anything else.** Change a harmless number (for example `HOOD_POSITION`),
   press **Enter** (or the check mark). On the Driver Station look for:

   `Dashboard edits received: 1 edits (last: HOOD_POSITION = 0.4, 1.2 s ago)`

   This line is on the INIT screen too (Shooter Bench), so you can prove the link before pressing
   START. On the INIT screen also check `Hardware: flywheel OK | hood OK | compression OK`.
   Verified on the real Control Hub on 2026-11-16: an edit typed in the browser was still there after a
   full page reload, so it reached the robot.

   - Count went up: Dashboard works, tune from the web page.
   - Count did not go up: the edit never left the browser. You did not press Enter/check, the laptop
     is not on the robot Wi-Fi, or the page is stale (reload it). The gamepad still works either way.
   - `Dashboard: NOT RUNNING`: Dashboard did not start in the app. Rebuild and reinstall the Robot
     Controller app, then reboot the Control Hub.
5. Graph tab: add `targetRpm` and `actualRpm` (shooter) or `txDeg`, `filteredTxDeg`, `demand` (aim).

## 1. Flywheel: "Shooter & Hood Bench Tuner"

Why the old tuning went badly: the old defaults had P (15) larger than F (12.5). On the REV hub,
F is about `32767 / max ticks per second` and P is about 10% of F. P = 15 means a 12% speed error
asks for 100% power, so the wheel hunts and no P ever looks right. This tuner measures F instead of
guessing and starts P at 0.1 F.

1. Start in **OPEN LOOP** (like the OnBot test). Cross/A spins at `OPEN_LOOP_POWER` (starts at 30%).
2. D-pad up/down raises the power in 5% steps. Run at the power you normally shoot at (about 90%).
3. After the speed holds still for 1.5 s the screen shows `STEADY`, the real rpm, battery voltage,
   a **Suggested F** and a rough max rpm. This replaces the "about 5000 rpm" estimate with a measurement.
4. **Right stick click** (or tick `APPLY_SUGGESTED_PIDF`): loads F, P = 0.1 F, I = D = 0.
5. **Back/Share**: switch to **VELOCITY** mode. Set `TARGET_RPM`, press Cross/A.
6. Tune P up in small steps (hold L2 + D-pad up, or edit `P` in Dashboard) until the `actualRpm`
   graph reaches `targetRpm` quickly without ringing. If it rings, lower P. Add D only if needed.
7. Feed balls. The `Last ball dip` line reports how many rpm the ball cost and how long recovery took.
   Use it to compare settings: smaller dip and shorter recovery is better.
8. **Options** (or tick `SAVE_CALIBRATION`) saves everything to the Control Hub. Copy the final
   P/I/D/F into `RobotHardwareConfig.flywheelPidf`.

Before the first run (Hood Servo Test robot configuration must be active on the Driver Station):
- Config names needed: motor `flywheel`, servos `hood` and `compression`, Ethernet device `limelight`.
- Pressing START powers both servos and moves them to the saved hood and compression positions
  immediately. Keep hands and wires clear and make sure the linkages are free.
- The flywheel never spins until you press A/Cross (or tick `FLYWHEEL_ON`). Hold B to stop.
- The hub config currently labels the flywheel as a 60 rpm, 2786 ticks/rev motor. If the real flywheel
  motor is a 28 ticks/rev bare motor, leave `TICKS_PER_REV` at 28 and ignore that line, but fix the
  motor type in the configuration when you can.

Check these lines if anything looks wrong:
- `Flywheel motor type`: the ticks per rev the robot configuration thinks the motor has. If it is not
  28, either fix the configuration or change `TICKS_PER_REV`. A wrong value makes every rpm number wrong.
- `PIDF on hub ... [SYNCED]`: the hub really holds the numbers you see. `MISMATCH` means the write failed.
- `WARNING: 0 encoder ticks/s`: the 4-pin encoder cable is not connected.
- B/Circle is the emergency stop (hold it).

Old saved PIDF/RPM values from the first version of this tuner are ignored automatically.

## 2. Auto-aim: "Limelight Auto Aim"

Assumes the Limelight is mounted **on the turret**: `tx = 0` means the shooter points at the tag.
If it is on the chassis this is not the right controller.

Works today, with the turret not attached:
1. In the Limelight web UI (`http://limelight.local:5801`), set pipeline 0 to **AprilTag** and note
   the goal tag ID. Set `TAG_ID_(-1_any)` in Dashboard to that ID (or leave -1 for any tag).
2. INIT the OpMode. `Limelight` section must show `Connected / running: true / true` and frames counting up.
3. Hold the tag in view and move it left and right. `txDeg` must follow, `AIM STATE` must go
   `TRACKING` then `LOCKED` as you centre it, and `demand` must be positive on one side and negative
   on the other. Pull the tag away: state becomes `SEARCHING` and demand is exactly 0.

With the turret attached (after the Turret Hardware Bench is complete):
The turret output is gated by the same safety layer as the Turret Hardware Bench. It stays at zero
until `TurretHardwareConfig` is filled in from the bench measurements (encoder backend and zero,
CR servo calibration, limits, sign checks). The screen line `Turret commissioning: NOT READY: ...`
names the first missing item. Do not edit those flags without measuring.
1. A arms, hold right bumper to move, B stops. The first motion test is a small tag offset, hand on B.
2. If the turret moves away from the tag, press LB (flips `AIM_SIGN`) and repeat.
3. Raise `AIM_KP` until it settles quickly, add `AIM_KD` if it overshoots. `AIM_OFFSET_DEG` trims
   a camera that is not perfectly aligned with the shooter.
4. Copy the final gains into the defaults in `VisionAimController.Config`.
