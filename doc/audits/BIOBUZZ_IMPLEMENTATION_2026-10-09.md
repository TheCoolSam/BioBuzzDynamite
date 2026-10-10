**BIOBUZZ audit repairs and integration — October 9, 2026**

The confirmed software defects have been repaired and the competition runtime
has been implemented. Hardware names and measurements remain **undecided**, as
confirmed by the user. `RobotHardwareConfig.measured()` therefore returns null;
the competition OpModes display **MOTION DISABLED** and map no devices. The
drivetrain and turret also have separate calibration/tuning gates. No hardware
was powered, connected, deployed to, or certified by this work.

**Resolution of the October 9 audit**

| Audit item | Code now present | Remaining dependency |
|---|---|---|
| Invalid time models | Unavailable/invalid estimates reject actions, preserving WAIT. Missing scoring poses and invalid model parameters also reject plans. Invalid probability data cannot become a measured zero. | Populate measured travel, full-load cycle and tip tables. |
| Stale robot state | Snapshot and acquisition timestamps, device health and monotonic reset generation survive mapping, estimation, history and automated decisions. Missing source provenance fails closed. | Verify localizer transactions and actual failure detection on the robot. |
| Duplicate/older camera evidence | Older/duplicate observations are rejected before association. Independent evidence growth uses an explicit measured cadence. Tracks are capped at 32, observations at six; assignment uses bounded subset dynamic programming. | HuskyLens exposes no unique frame identity through this SDK. Poll timing cannot prove that its internal image changed; characterize repetition/latency and lighting. |
| Consumption/reset lifecycle | Confirmed consumption has spatial suppression of delayed evidence. Localization resets clear histories, tracker, plan, turret reference and HIVE readiness together. Confirmed intake invalidates strategy even if aggregate counts later match. | Validate real entry/exit identification and event timing. |
| Mesh validation | Empty/missing selections, malformed files, nonfinite/degenerate faces, open/disconnected shells, winding and negative volume fail under normal and optimized Python. Recursive and manifest selection are supported; stale success reports are replaced before validation. | Topology does not certify STL units, physical fit, self-intersection, material strength or mechanism clearance. |
| Competition integration | `RobotCoordinator`, `RevRobotIO`, `BioBuzzTeleOp` and `BioBuzzAutonomous` provide one actuator owner, coherent snapshots, cancellation and visible fault reasons. Independent final I/O checks enforce fresh reads, finite outputs, caps and servo bounds. | Complete and bench accept the physical configuration. Competition odometry currently supports Pinpoint health checks; other selected localizers fail closed. |
| Turret control | Measured position/velocity references with speed/acceleration bounds, measured velocity feedback, guarded REV competition commands and driver-priority yaw assist. Aim readiness checks the actual geometric error, using measured pivot offsets. | Select absolute acquisition, limits, signs, dynamics and stopping margins. PWM capture remains unavailable until an actual capture interface/protocol is selected. |
| Scoring/inventory | Ordered confirmed queue, capacity reservations, sensor dwell filtering, measured ball-specific recipes/ranges, staging, stable RPM, settled servos, stationary/aim/HIVE gates, one-piece feed, exit confirmation, bounded recovery and reacquisition timeouts. Partial release faults mark inventory uncertain; cancellation cannot clear a fault. | Characterize sensors, motor powers, flywheel PIDF, servo positions, recipe range and timing. Verify one physical piece per pulse and HIVE classification. |
| Vision | Background floor camera acquisition, calibrated floor projection/history, Limelight target identity/pipeline/freshness filtering and optical-to-turret/chassis/field transforms at exposure time. Moving target observations never correct odometry. HIVE readiness is cleared after each confirmed release. | Measure camera frames, extrinsics, latency, target-to-cell geometry, alliance IDs and classifier behavior for both settled sides and tipping. |
| Feasible pickup and shot contracts | Enclosing body/intake radius, expanded rectangular obstacles, staged capture segment, selected scoring pose retained in `PickupPlan`, and measured full-load cycle timing. Held pickup assist executes one capture or scoring-position move and stops on faults/timeouts. | Supply measured geometry, corridors, accepted scoring poses and motion gains. The circle/rectangle test is conservative; it does not predict other moving robots. |
| Autonomous/runtime evidence | Explicit route and park waypoints; confirmed pickup/release advancement; per-leg deadlines, early park and hard 30-second stop; retained early pickup events; reset/clock faults. Loop percentiles and inhibit reasons are logged. | Run timed Control Hub tests under crowded scenes, communication faults, jams and actual match routes. Desktop tests and packaging do not establish a real-time deadline or physical stopping distance. |

**Configuration to supply after hardware selection**

1. In `pedro/PedroConstants.java` and `PedroFactory.java`, replace placeholder
   names, motor/pod directions, offsets, scales, speed/braking limits and feedback
   models with measurements. Set `HARDWARE_AND_TUNING_CONFIRMED` only after sign,
   distance, rotation and stop tests. The new routines request manual drive
   powers through their measured waypoint controllers; Pedro's placeholder
   path-following tune must still be replaced before using follower paths.
2. In `turret/TurretHardwareConfig.java`, select a supported absolute signal,
   enter transfer calibration, output ratio, zero, encoder/motor signs, valid
   rate, verified travel and bench caps. Supply a measured `TurretControlProfile`
   and complete bench readiness/tuning flags. Keep PWM unselected while no pulse
   capture implementation is defined. Test startup from multiple angles and
   cables, stops, faults, emergency stop and rearming.
3. In `robot/RobotHardwareConfig.java`, return a freshly constructed `Settings`
   with real device names, directions, active sensor levels and measured dwell.
   `EntryClassifier.initialize(HardwareMap)` maps the identification sensor once;
   `classify()` performs its read inside the timed I/O snapshot and returns null
   for an unknown type. Confirm physical entry order and preload contents.
   `confirmedPreload` is an explicit inspected queue; the code assumes no pieces.
4. Enter encoder ticks per flywheel revolution, maximum characterized RPM, REV
   velocity PIDF, accepted mechanism caps and intake/feed/reverse powers. Supply
   `ShotRecipe` entries with measured RPM, hood/compression positions, settling,
   RPM tolerance/stability and minimum/maximum target range. The fixed-delay and
   unbounded-range constructors remain simulation fixtures and cannot satisfy
   competition configuration validation.
5. Enter floor calibration and taught IDs/alliance. Use
   `PieceTrackerConstants.withMeasuredTiming(latency, independentEvidenceInterval)`
   to explicitly record camera timing. Supply a measured right-handed 3x3 optical
   rotation and camera/pivot offsets, Limelight pipeline/alliance IDs and a
   classifier that can recognize settled sides, TIPPING and UNKNOWN. A tag ID
   alone does not identify the settled HIVE side.
6. Supply `SweptCaptureFeasibility` with the enclosing radius of every enabled
   robot/mechanism pose and field obstacle rectangles. Supply
   `PreferredPoseShotSetup` with accepted scoring poses, measured travel/yaw
   rates, footprint and a full-load `CycleTimeModel`. Unknown load cycles should
   return NaN. Include feed, recovery/reacquisition and tip settling in the
   measured cycle. Supply shot position/heading tolerances and measured firing
   motion tolerances. Add `PickupAssist.Settings` when that motion is accepted.
7. Supply the autonomous starting pose, explicit approach/capture/scoring route,
   park point, measured waypoint gains/caps/tolerances/timeouts and park margin.
   Use separate approach and `collect=true` capture waypoints, and separate
   `shoot=true` scoring waypoints. Park waypoints cannot collect or shoot. A
   scoring fault can trigger park only with usable localization and a clear
   corridor; a localization or hardware fault stops movement.

**Driver operation and recovery once configured**

- Hold gamepad 1 RB to enable. A starts one shot; LT requests intake; B cancels;
  LB permits chassis unwind with driver rotation taking priority. Releasing RB
  stops outputs before potentially slow hardware reads.
- Hold X with neutral sticks to execute one planned pickup, or move to the
  selected scoring pose for SHOOT_NOW. It never fires the feeder itself. Release
  and press again to obtain the next fresh plan. Stick movement or A cancels
  that assist. The coordinator checks the captured piece's expected type.
- A confirmed exit removes exactly the front piece. A held shot trigger never
  automatically starts the next release. A new settled post-release target/HIVE
  observation is required. Timeouts never count as an exit.
- After an ambiguous release or unexpected entry/exit, physically inspect the
  queue with outputs disabled. On gamepad 2, Y clears the recovery draft, UP
  appends pollen, DOWN appends nectar in feed order, X removes the last entry;
  BACK+START explicitly confirms the inspected draft. Then neutral gamepad 1
  BACK+START, with RB released and no intake/shot demand, resets controller
  faults. Hardware write faults require stopping and reinitializing the OpMode
  after repair. Digital sensors and plausible analog voltage cannot prove that
  wiring is intact; validate disconnect/stuck-signal behavior physically.
- Fixed-route autonomous accepts B as a latched driver cancellation. It also
  stops at 30 seconds even if its route or shot remains incomplete.

**Verification**

Run `tools/run-desktop-tests.ps1`, `python -m unittest discover -s tools/tests -v`,
and `gradlew.bat --offline --project-cache-dir build/audit-gradle-cache :TeamCode:assembleDebug`.
Final observed results after the last runtime changes:

- **1,760 Java checks passed** across nine suites: planner 99, tracker 180,
  state history 23, turret safety 72, Pedro mapping 45, safety regressions 498,
  turret bench 114, audit regressions 38 and robot integration 691.
- **11 Python test methods passed**, with each validation case exercised under
  normal Python and `python -O`. Includes empty/recursive/manifest selection,
  malformed/degenerate geometry and topology failure reporting.
- **Android debug build succeeded**, including SDK-dependent compilation and
  APK packaging at `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
- `git diff --check` passed; all 15 Fusion helper Python files parsed successfully.

Python tests use temporary directories inside `build` and
verify their cleanup paths. The Android build includes SDK adapters and both
competition OpModes; desktop scenarios inject simulated measurements and sensor
events. Compiler/Gradle deprecation warnings remain in the upstream toolchain.

The CAD/print assembly work from the audit remains physical work: horn/link pin
retention, compression travel stops and shaft retention, rail attachment for
the 80T gear, and loaded sliding/servo tests. No dimensions or print geometry
were changed to substitute for these undecided hardware choices. Existing
staged and uncommitted user work was preserved.
