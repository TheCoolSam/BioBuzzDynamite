# Tuning quickstart: shooter flywheel and Limelight auto-aim

Both OpModes are in the **Bench** group on the Driver Hub and take tuning settings in FTC Dashboard.

## 0. Connect Dashboard (do this first, every session)

1. Laptop on the Control Hub Wi-Fi (`FTC-xxxx`).
2. Browser: `http://192.168.43.1:8080/dash`
3. INIT the OpMode. In the **Configuration** tab a new group appears:
   `Shooter Bench` or `Auto Aim`. It disappears when the OpMode stops.
4. **Test the link before anything else.** Change a harmless number (for example `HOOD_POSITION`),
   press **Enter** (or the check mark). On the Driver Hub look for the edit count to go up.

   **Shooter Bench** shows `Edits received` during INIT. Check that `flywheel`, `hood`, and
   `compression` all show **OK** there.
   Verified on the real Control Hub on 2026-11-16: an edit typed in the browser was still there after a
   full page reload, so it reached the robot.

   - Count went up: Dashboard works, tune from the web page.
   - Count did not go up: the edit never left the browser. You did not press Enter/check, the laptop
     is not on the robot Wi-Fi, or the page is stale (reload it).
   - `Dashboard: NOT RUNNING`: Dashboard did not start in the app. Rebuild and reinstall the Robot
     Controller app, then reboot the Control Hub.
5. Graph tab: add `targetRpm` and `actualRpm` (shooter) or `txDeg`, `filteredTxDeg`, `demand` (aim).

## 1. Flywheel: "Shooter & Hood Bench Tuner"

1. Check the configuration names `flywheel`, `hood`, and `compression` in INIT. Press START
   with the linkages clear. The gamepad D-pad moves the hood (up/down) and the selected ball's
   compression (left/right).
2. In Dashboard under **Shooter Bench**, check `TICKS_PER_REV` and set `TARGET_RPM` to your
   shot speed. Set `AUTO_TUNE_PIDF` with the flywheel off and stopped.
3. Wait for **Done**. The flywheel stops and the verified gains load into velocity mode.
   If tuning fails, read the reason in Dashboard. Set `AUTO_TUNE_PIDF` again during a run to cancel.
4. Set `FLYWHEEL_ON` and graph `targetRpm` and `actualRpm`. Once speed settles, test shot recovery.
   Dashboard shows the last speed dip and recovery time.
5. Turn off `FLYWHEEL_ON` and set `SAVE_CALIBRATION` to keep the settings on the Control Hub.
   Use `STOP_ALL` to stop the flywheel and release both servos, or press **STOP** on the Driver Hub
   to end the OpMode.

For manual tuning and servo calibration, see [Shooter tuning guide](SHOOTER_TUNING_GUIDE.md).

## 2. Auto-aim: "Limelight Auto Aim"

Assumes the Limelight is mounted **on the turret**: `tx = 0` means the shooter points at the tag.
If it is on the chassis this is not the right controller.

### BIOBUZZ AprilTags (Competition Manual section 9.9)

- 36h11 family, **3.25 in = 82.55 mm** (set the Limelight `Marker Size` to 82.55).
- Four 4-tag clusters, one under each CELL. They move when the HIVE tips, so the tag you see changes
  during a match. They are **not** for field localization.

| CELL | Tag IDs |
|---|---|
| Red, far side (opposite audience) | 30, 31, 32, 33 |
| Red, audience side | 34, 35, 36, 37 |
| Blue, audience side | 38, 39, 40, 41 |
| Blue, far side (opposite audience) | 42, 43, 44, 45 |

Red alliance targets **30 to 37**, blue alliance targets **38 to 45**. Verify these against the
current manual (Section 9.9) before competition. The manual is updated through TU releases.

How to make sure the Limelight looks for the right tags (the pipeline's ID Filters box decides what the
robot is allowed to see; a tag outside the list is drawn on the stream but never reported):
1. Open the Limelight page, pipeline 0, **Configuration** tab (`http://192.168.43.1:5801` when your
   laptop is on the Control Hub Wi-Fi).
2. Family `AprilTag Classic 36h11`, Marker Size `82.55`.
3. **ID Filters**: red `30,31,32,33,34,35,36,37`, blue `38,39,40,41,42,43,44,45`. Or leave it blank and pick
   in code with `TAG_ID_(-1_any)`. (`20,24` in that box is a leftover from last season.)
4. Check: show tag 34 (red) or 38 (blue) on a phone. The stream draws it and `tx`/`ty`/`ta` move.
   Then show a tag outside your list: it must NOT produce `tx`/`ty`.
5. Always re-check it after you reload a pipeline or swap Limelights.

Works today, with the turret not attached:
1. In the Limelight web UI, set pipeline 0 to **AprilTag** and use the IDs above. Set `TAG_ID_(-1_any)`
   in Dashboard to one of them, or leave -1 for any tag in the filter.
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
