# BioBuzz: code review, autonomous plan, and aiming at the top of the hive

Written 2026-11-15. Nothing here has been run on the robot. Game geometry (hive opening size, heights,
scoring rules) is taken from the repo's own audits, **not** from the official manual, which I couldn't
download. Check every field number against the manual before trusting it.

---

## 1. The big picture

The repo has a very safety-minded competition stack (`BioBuzzTeleOp`, `BioBuzzAutonomous`,
`RobotCoordinator`, `ScoringController`, `HiveTracker`, `FixedRouteAuto`, Pedro, turret safety gate).
The design is **all or nothing**: if any one "measured" value is missing, the whole stack refuses to
start. Today many are missing, so **none of the competition OpModes can start**. That is by design,
but it means the real work is measuring, not coding.

### What stops each OpMode from starting today (verified in code)

| OpMode | Blocked by |
|---|---|
| `BIOBUZZ TeleOp` / `Stationary Scoring` | `RevRobotIO` builds `RevTurretIO`, which throws until `TurretHardwareConfig` flags + calibration are filled from bench measurements. `RobotHardwareConfig.validate()` also demands many measured values. |
| `BIOBUZZ Fixed Route` (auto) | Same as above, plus `PedroFactory.create` throws until `PedroConstants.HARDWARE_AND_TUNING_CONFIRMED` is true. |
| `BIOBUZZ Drive Practice` | `@Disabled` in the working tree (uncommitted), and motor-reverse flags in `DriveHardwareConfig` are all `false`. |
| `Pedro Integration Test`, `HuskyLens Piece Test` | `@Disabled` in the working tree. |

So right now **there is no enabled OpMode in this repo that drives the robot**. If you drive from OnBot
Java, that's fine, but anything you want to test before comp needs a drive path.

---

## 2. Code improvements, in priority order

### P0. Fix before anyone trusts the numbers

1. **Tag 2 can never produce a target.** The placeholder cell points are
   `(tag 1, SIDE_A_UP)` and `(tag 2, SIDE_B_UP)`, but `s.cellClassifier` is hard-coded to return
   `SIDE_A_UP` for every tag. Looking up `(tag 2, SIDE_A_UP)` finds nothing, so tag 2 is dropped.
   The classifier must decide the hive state from what the camera actually sees (see section 4).
2. **"Measured" values that are placeholders.** `RobotHardwareConfig` hard-codes
   `benchAccepted = true`, uses identity camera rotation, a synthetic launcher geometry grid, shot-table
   recipes of 1200/1600 rpm with made-up release data, and identical cell offsets for both tags. If someone
   flips the turret safety flags, the system will happily trust all of that. Make placeholders fail
   validation (e.g. `benchAccepted = false` until a real bench run exists) so "configured" can't be
   reached by accident.
3. **Shot table RPM vs reality.** Your OnBot tests were around 90% power (~5000 rpm is an estimate).
   The table says 1200/1600. Use the Shooter & Hood Bench Tuner to measure real rpm for each shot,
   then write those into the table.
4. **Pedro is not configured at all.** All four motor directions are `FORWARD`, pod offsets are `0`,
   max speed is `1 in/s`, feedback gains are `0`. This is fine as a placeholder but means no path
   following. Details in section 3.

### P1. Make it easier to test piece by piece

5. **Split "all or nothing" into stages.** Add small OpModes that each need only one subsystem:
   a plain drive TeleOp (mecanum + intake + shooter on the gamepad, no turret gate), a shooter-only
   test (already have it), a turret-only test (have it), a vision-only test (have it). Combine them only
   once each works. The competition stack should be the *last* thing you bring up, not the first thing
   you need.
6. **`tools/run-desktop-tests.ps1` doesn't run at HEAD.** It compiles SDK-dependent files
   (`ShooterCalibrationStorage`, `HuskyLensEntryClassifier`, `DriveHardwareConfig`/`PedroConstants`)
   that need the Android SDK. Fix it by compiling only the pure-logic packages, or by running the tests
   through Gradle. `VisionAimScenarios` and `AimPointScenarios` run standalone today
   (`javac` the two files and `java` the class).
7. **Tunables in one place.** The bench OpModes now use FTC Dashboard through `DashboardTunables`.
   Move the competition values you tune often (shot rpm, hood, compression per shot) to Dashboard
   too, then save to `ShooterCalibrationStorage` once they're good.

### P2. Nice to have

8. Log every shot (rpm before, dip depth, hood, compression, range, hit/miss). `ShotJournal` already
   records HIT/MISS/TIP from gamepad2; make sure it writes to a file you can pull from the robot.
9. Battery-voltage-compensate flywheel feedforward (F is not voltage compensated; a sagging battery
   makes the same F give lower rpm).
10. Keep the committed tree clean: ~100 untracked CAD/Python/JSON files at the repo root make real
    changes hard to review. Move them under `tools/` or add them to `.gitignore`.

---

## 3. A good autonomous for BioBuzz

I can't give you field coordinates, because I don't have the verified field drawing. What I can give is
the order to build it in. **Each tier is worth points on its own; don't start tier N+1 until N works
five times in a row.**

**Tier 0: Get the drive working (prerequisite).**
- Run the motors one at a time, set the correct reverse flags in `DriveHardwareConfig` and
  `PedroConstants`, and confirm that stick-forward drives forward.
- If you use the goBILDA Pinpoint: measure the two pod offsets from the robot's center, set them in
  `PedroConstants`, and set the encoder directions.
- Then run Pedro's tuners (localizer, then velocity/deceleration, then translational/heading PIDs).
  Until this is done, `FixedRouteAuto`'s plain P-controller is the only path follower you have, and it
  has no deceleration limits.

**Tier 1: Leave the starting zone (points almost every match).**
Drive straight off the line and park. This only needs the drive working. Time it, add a safety margin,
and make it the fallback for every other auto.

**Tier 2: Preload shot from a fixed spot.**
Drive to one marked tile position, spin up the flywheel *during* the drive, aim the turret (vision if
working, fixed angle if not), fire the preload. A fixed-angle fallback is better than no auto when the
Limelight has a bad day. Use the Shooter Bench to find rpm/hood for exactly that spot.

**Tier 3: Preload plus one pickup and shoot.**
Add a pickup waypoint with the intake running, return to the shot spot, fire again. Note
`FixedRouteAuto` supports collect/shoot waypoints and a park deadline; fill its route with real
coordinates once Pedro is tuned.

**Tier 4: Cycles.**
Repeat tier 3 while time remains, keeping the flywheel spinning between shots. Park with a deadline
(`autoParkAt`) that leaves enough time to be safe.

**Rules of thumb**
- Always store the end pose in `PoseStorage` so TeleOp starts with a known pose.
- Choose a starting position you can place the robot at repeatably (against a wall or a tile edge).
  A 1-inch placement error is a 1-inch shot error.
- Make the auto robust to the Limelight failing: if no tag after N ms, shoot at the pre-measured angle.
- Build separate Red and Blue versions by mirroring the field, not by retuning.

---

## 4. Aiming at the top of the hive (and what backspin does)

### Why aim point matters

The Limelight's `tx` points the turret at the tag's **center**. The scoring opening is not at the tag
center, and on BioBuzz the tags are mounted on the CELLS, which **tip**, so the opening moves relative to
the tag. You want the turret on a point you choose (for example "just above the opening, toward the back
of the hive"), expressed in the tag's own frame so it follows the tag as the cell moves.

The repo already has this idea: `CellTargetGeometry` stores a per-tag, per-hive-state offset point
(x, y, z in the tag frame, plus a lip clearance). The problem is the numbers are placeholders and the
hive-state classifier is constant.

### What backspin does (my physical reasoning, not measured)

- Backspin produces a Magnus lift force, so the ball flies a bit higher and farther than the same
  speed without spin.
- When a ball with backspin lands on a horizontal surface, the spin kicks it **back toward the shooter**.
  That's the rebound problem. Against a vertical wall, backspin tends to push the ball **down** instead.
- The fixes, in order of impact: (1) enter the opening at a steeper angle by adding hood pitch, so the ball
  drops in rather than skimming across; (2) aim at a point that leaves lip clearance, not at the lip;
  (3) reduce spin (less compression squeeze or a different wheel-to-ball speed ratio) if the shots still
  come back out.

The only reliable answer is experiments. A starting protocol:

1. Fix the robot at one distance. Fix rpm and compression. Fire 10 shots at the tag centre. Record HIT / MISS / TIP and
   where the misses land (short, long, bounced back out).
2. Raise the aim point 1 inch, fire 10 more. Raise again, repeat. Keep the offset that gives the most HITs and fewest rebounds.
3. Repeat at 2 or 3 other distances. If the best offset changes with distance, scale it (a fixed number of
   inches is fewer degrees farther away, as `AimPointScenarios` shows).
4. Then vary the hood angle and compression at the best offset.

### New in this commit: `AIM_AT_POINT` in the Limelight Auto Aim OpMode

`AimPoint` (pure math, tested in `AimPointScenarios`) computes the bearing, elevation and range to a point
offset from the tag, expressed in the tag's frame and rotated by the tag's measured orientation.

In the Dashboard category **Auto Aim**:

- `AIM_AT_POINT`: off = aim at tag center (old behaviour); on = steer yaw to the offset point.
- `POINT_TAG_X_IN`, `POINT_TAG_Y_IN`, `POINT_TAG_Z_IN`: the point in the tag's frame, inches.

The telemetry shows tag-centre vs aim-point yaw, elevation, range, and the raw tag pose.

**Verify the signs on the robot; I did not assume them.** With the turret not moving and a tag in view:
1. Set `POINT_TAG_Y_IN` to 4. Watch "Elevation: tag centre / aim point". Whichever way it moves tells you
   whether +Y on the tag is up or down. Flip the sign if needed. Repeat for X.
2. Check that "Yaw: tag centre" matches "Limelight tx" within about a degree. If it doesn't, the pose
   angle ordering in the SDK or the camera is not what the code assumes.
3. Turn the tag by hand about its vertical axis and see which of pitch / yaw / roll changes.

**Limits.** This computes yaw only; elevation and range are displayed so you can choose hood angle and rpm
from them, but nothing sets the hood from them yet. The hive state (which side up) still has to be
detected before the offset is correct for both states. Aim point is yaw-only because the turret only rotates.

### To make this real in the competition code

1. Replace the constant `cellClassifier` with a real one. Use the tag's measured orientation or height
   in the camera (a tipped cell changes the tag pose in a way you can read directly) and check it against
   several tipped/untipped examples.
2. Measure the cell points from the CAD and the real field element, per tag and per state, and put them
   in `RobotHardwareConfig` (they should differ per tag and per state).
3. Measure the camera's mounting rotation and position on the turret and fill in the camera transform
   (it is identity today).
4. Use `ShotJournal` (gamepad2 A/B/sticks) to log HIT/MISS at each offset, then update the offsets from the data.

---

## 5. Suggested order for the next few sessions

1. Attach the turret. Run **Turret Hardware Bench**; fill `TurretHardwareConfig` from measurements.
2. **Shooter & Hood Bench Tuner**: measure F, then P, then ball-dip, then save.
3. **Limelight Auto Aim** with `AIM_AT_POINT` off: tune kP/kD with the turret in place.
4. Turn `AIM_AT_POINT` on, verify signs, run the backspin experiment above.
5. Drive: reverse flags, Pinpoint offsets, Pedro tuners. Tier 1 auto, then tier 2.
6. Only then: replace placeholders in `RobotHardwareConfig` and try the competition OpModes.
