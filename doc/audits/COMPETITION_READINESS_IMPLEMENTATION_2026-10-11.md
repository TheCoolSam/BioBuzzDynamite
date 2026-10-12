# Competition readiness implementation — October 11, 2026

Implements priorities 2–5 from the review's user-facing list: autonomous shot requests, legal wall departure, independent driving/stationary scoring, and fault recovery/measurement. The existing pending launcher calibration, feedback, Axon and logging changes are retained as dependencies. No CAD, print, Fusion tooling or physical calibration values are changed.

## Autonomous shot actions

Each shooting waypoint has a positive action ID. The route keeps requesting that ID until the scorer accepts it and a matching sensed exit completes it, or a fault/deadline cancels the route. Missing target/range and WAIT_FOR_HIVE no longer consume the only start edge. Accepted IDs cannot be replayed, including after a cancellation. Driver A remains an edge-triggered ordinary shot.

`ScoringController.acceptedRequestId()`, `completedRequestId()` and `failedRequestId()` expose the latest action lifecycle. `FixedRouteAuto` uses matching completion IDs, not an inferred inventory-count decrease. Interrupted release still retains possible flight evidence, marks inventory uncertain and prevents automatic retries. A completed ID confirms release, not a scored ball or tipped hive.

## Measured wall departure

Supply `RobotHardwareConfig.Settings.departure` when the starting footprint needs a constrained departure from the wall. Its endpoint, heading, half-forward/half-left dimensions, heading tolerance and timeout must be physically verified. Dimensions enclose every mechanism/ball pose used during departure; the enclosing circle remains at least their diagonal radius.

Departure commands translation with zero chassis yaw, inhibits intake/launcher/yaw outputs, checks actual heading on each update, uses the oriented rectangular wall extent, and retains the full enclosing-circle obstacle clearance. The endpoint must have full ordinary swept clearance. Heading drift, invalid pose, reset, obstruction or timeout stops the routine. After reaching full clearance, normal route timing begins; the overall autonomous deadline still includes departure. This is a constrained fixed-heading departure, not a general oriented path planner. Bench-test mechanical retention and drift before acceptance.

## Independent modes and profiles

**BIOBUZZ Drive Practice** uses only `DriveHardwareConfig.measured()`. Supply a `BasicDrive.Settings` containing four unique names in FL, BL, FR, BR order, measured reversal flags, a measured power cap, and verified signs/stop behavior. It maps four motors, uses brake/without-encoder operation, and requires no localizer, path tune, camera or launcher. Default configuration remains null. Verify that unpowered mechanisms are physically restrained before driving this standalone mode.

Drive Practice controls: G1 RB enables; B stops; left stick translates relative to the robot; right stick turns. Invalid/nonfinite demands are refused. Failed motor writes latch a fault and attempt to stop every mapped motor. Reinitialize after a write fault.

**BIOBUZZ Stationary Scoring** loads `RobotHardwareConfig.stationaryMeasured()`. This profile requires a calibrated physical launcher, output-side pitch/gap sensing, measured yaw control, localization, inventory sensing, internal cell geometry, shot table, flight/release bounds and `MotionShotSolver.StationarySettings`. Set moving-shot settings and burst policy to null. The stationary solver uses the pitch/gap-dependent release point and rejects unsupported height/range, excessive motion and miss bounds. A measured stable window is required before feeding; readiness is still enforced throughout release. Residual motion, localization/target error, launcher spread and geometry error remain part of the acceptance budget.

Pickup planning is separately enabled by `pickupPlanningEnabled`. Leave it false for stationary commissioning: no HuskyLens worker, floor projection, tip model or pickup-cycle model is needed. Enabling it requires all measured planning/vision dependencies. Autonomous still needs a measured route and footprint. Normal field-centric competition driving still requires verified Pedro configuration; Drive Practice provides the independent route to early chassis testing.

## Deliberate drive-only recovery

Snapshots now report independent drive, localization, yaw, launcher and inventory health. Normal competition operation first stops and latches on unhealthy required sensing. An inventory sensor failure marks the queue uncertain; loss during release retains the existing reconciliation requirement. Target-vision exceptions produce no target and inhibit firing.

Configure `Settings.recoveryDrive` only from verified four-motor settings matching the competition motor names and reversal flags. After a fault, release RB, center the sticks, release shoot/intake/pre-spin, then press G1 BACK+Y. Recovery is refused without measured drive settings or if a stop/write fault leaves outputs uncertain. Follower update/write failures remain global faults; a shared-drive output failure cannot be bypassed.

Accepted recovery latches **robot-centric driving only** until OpMode reinitialization. Hold RB to drive; B/releasing RB stops. All automatic aiming, intake, transfer, feeder, RPM and new positional-servo commands are inhibited. The failed localization and mechanism sensors are not read on the recovery control path, and floor acquisition is stopped. The existing motor output cap, finite-input and timing gates remain. Scoring faults and unknown inventory are not erased by recovery.

## Timing and observed outcomes

TeleOp logs complete callback duration after logging and telemetry construction, plus start-to-start intervals that include framework/scheduling overhead. The coordinator reports read, optional planner, write and total cycle durations; failed transactions retain their elapsed stage time. Rolling p50/p95/p99 and lifetime maxima remain bounded by the existing metrics storage. These measurements do not imply a Control Hub timing result until the robot is run.

Every shot CSV row now appends `runId` to the existing schema, alongside configuration revision and shot ID. G2 observer controls label the displayed **last confirmed release**: A = HIT, B = MISS, left stick click = TIP, right stick click = NO_TIP. Button edges prevent repeated labels while held; opposing labels pressed together are ignored. Hit and tip labels are independent rows, so missing observations remain unknown. External video labeling remains available through `ShotJournal.outcome(...)` for any known shot ID. An exit sensor never automatically marks a hit or tip.

## Validation

Final verification: **2,843 checks passed across 12 desktop suites**, including
98 competition-readiness checks. **Android `:TeamCode:assembleDebug` succeeded**.
`git diff --check` passed. Existing Java 8 target/Gradle deprecation warnings remain.

The desktop suite includes `CompetitionReadinessScenarios`: pending/accepted/completed/cancelled action IDs, late targets, busy scoring, matching route completions, legal wall starts, drift/obstacles/timeouts/resets, capped basic driving, deliberate recovery and mechanism suppression, stationary physical geometry/range/height/motion gates, integrated stationary feeding without advanced dependencies, and shot/run outcome identity.

Run `tools/run-desktop-tests.ps1` and `gradlew.bat :TeamCode:assembleDebug --offline --no-daemon`. Desktop scenarios are synthetic; SDK transaction failures, neutral/stopping behavior, mechanism retention, physical feedback, shots and recovery driving require hardware acceptance. All measured profile factories remain disabled pending the team's real measurements.
