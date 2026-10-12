# FTC competition readiness review — October 11, 2026

**Implementation follow-up:** The autonomous, departure, independent mode,
recovery and measurement work requested after this review is described in
[the implementation record](COMPETITION_READINESS_IMPLEMENTATION_2026-10-11.md).
Findings and verification below describe the original review snapshot.

The first competition is December 5, 2026, 55 days away. The team has not built the physical robot. The best next investment is a complete, serviceable robot that can collect, shoot and finish matches repeatedly, with enough time left for driver practice. This review identifies software work that supports that outcome; it cannot establish physical performance or predict a Worlds qualification.

Scope: current working tree, including the team's uncommitted launcher changes. Competition OpModes, hardware adapters, drive/localization integration, scoring, target vision, pickup planning, calibration templates and desktop scenarios were inspected. Robot behavior code was not changed. This review adds documentation and a standalone reproduction probe.

## What is already worth keeping

- A central actuator owner, immutable snapshots, freshness checks and localization reset handling.
- Confirmed ordered inventory, sensed exit confirmation, bounded jam recovery and uncertainty after interrupted release.
- Absolute yaw sensing, output limits, an Axon CR adapter and calibrated neutral handling.
- Separate measured pitch/compression readiness, physical actuator maps and geometry-dependent release offsets.
- Ball-specific range/flight/release tables that reject unsupported ranges and heights.
- Exposure-time target projection, an internal cell aim point, motion uncertainty gates and conservative in-flight tracking.
- Deliberate pre-spin, optional intake during shooting after physical feed isolation is verified, and a bounded shot journal.
- Bounded tracking work and a planner that runs at lower cadence than mechanism control.

These features are present in the current source. Older audit recommendations about adding an Axon adapter, physical launcher feedback, pre-spin or shot logging have already been addressed in software. Their physical calibration remains outstanding.

## Highest-priority findings

### 1. No deployable measured robot profile exists — commissioning blocker

`robot/RobotHardwareConfig.java:18` returns null. Both competition OpModes intentionally remain powerless. `pedro/PedroConstants.java:17` is also false; motor directions, localizer offsets, speeds and braking are explicitly placeholders. `pedro/PedroFactory.java:155` has zero feedback coefficients. `turret/TurretHardwareConfig.java:6` has no selected encoder backend and `:10` has no CR calibration. Both shot CSV files contain only headers.

Action: build and commission the drivetrain and shooter; record verified signs, hardware names, geometry, feedback transfer curves, yaw stopping behavior, release timing and shooting measurements in a versioned profile. Do not enable flags by replacing missing measurements with desktop fixture values. Verify the selected odometry device's encoder scale as well as offsets and signs. The current competition adapter supports Pinpoint health specifically.

### 2. Autonomous can silently lose its only shot request — reproduced, P1

`BioBuzzAutonomous.java:41` sends one start pulse on entering SHOOT. `scoring/ScoringController.java:169` accepts a start only in IDLE; `:171` returns immediately when no calibrated selection is available. `robot/FixedRouteAuto.java:69` then waits for an exit without checking whether the scorer accepted the request.

Two triggers: the target/range is unavailable on the first SHOOT cycle, or the next shooting waypoint is reached while the previous shot is in WAIT_FOR_HIVE. In each case the only pulse is consumed. Later target recovery/scorer readiness does not generate another request; the waypoint eventually times out toward park.

Action: give autonomous shot requests an explicit accepted/completed/faulted lifecycle, ideally an action ID. Retain a pending request until safely accepted or its deadline expires. Never automatically replay a release with ambiguous inventory. Verify delayed vision, consecutive shooting waypoints and scorer recovery together through the OpMode adapter.

### 3. The conservative footprint can prevent a legal start from leaving the wall — reproduced, P1

`planning/pickup/SweptCaptureFeasibility.java:27,35` requires both endpoint centers to remain at least the enclosing radius from every wall. `robot/FixedRouteAuto.java:66` applies that test before any drive output.

For an illustrative 18-inch square robot touching a wall, the center is 9 inches from that wall, while a correctly enclosing rotation-independent circle has radius sqrt(9² + 9²) = 12.73 inches. The model rejects even a straight inward retreat and autonomous immediately latches FAULT. This is a demonstrated limitation for rectangular wall-contact starts; the actual robot's shape is not known yet.

Action: add a verified departure segment using the actual oriented starting footprint, or use an oriented footprint and swept-motion check. Restrict yaw during the departure if necessary. Preserve conservative clearance after departure. Reducing the circle below the measured enclosing radius would conceal collisions.

The official BIOBUZZ TU04 manual requires a wall-contact starting position and contact with four pollen preloads (G304). Preload inventory must describe what is actually carried; floor-contact preloads should not be counted as carried before collection. [Official manual](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU04.htm)

### 4. Basic driving depends on completing the entire advanced configuration — architecture gap

`RobotHardwareConfig.Settings.validate():65–79` requires motion-shot settings, launcher geometry and output sensors, both shot tables, floor vision, a tip model and a measured pickup/scoring cycle before the integrated OpMode can initialize. The fixed-recipe/stationary branches in TeleOp exist, but this validator makes those simpler configuration paths unavailable.

Action: add explicit commissioning capabilities: drivetrain-only practice, launcher bench/calibration, and calibrated stationary competition scoring. Each should validate every dependency it actually uses. A stationary mode must retain measured launcher readiness, inventory, target and flight-safety checks. This lets the team practice driving and characterize mechanisms while advanced features are still being measured.

### 5. Some subsystem faults disable all mobility — competition resilience tradeoff

`robot/RevRobotIO.java:101` combines localization, turret sample validity and flywheel RPM into one health flag. `robot/RobotCoordinator.java:96` stops everything and latches a fault when that flag fails. SDK read/write exceptions also take the global stop path. A turret encoder failure can therefore leave a mechanically healthy drivetrain unable to continue a match.

Action: separate drive, localization, yaw, launcher and inventory health. Provide deliberate driver recovery to a measured basic drive mode when drive hardware is healthy. Keep autonomous and automatic aim disabled without localization; keep feeding disabled on launcher/inventory faults. Shared hub failures and uncertain actuator writes still need an overall stop. Test every transition with injected faults; do not simply remove health checks.

### 6. Reported loop timing omits part of the loop — confirmed measurement gap

`BioBuzzTeleOp.java:102` records elapsed work before the periodic summary/log at `:103` and telemetry at `:108`. That number is neither total loop work nor the interval between control cycles. Planning can enumerate 5,861 candidate sequences for ten candidates and four available slots; it runs synchronously every 0.1 seconds in the coordinator. Tracking is capped, but allocation, planning, logging and SDK latency still need measurement on the Control Hub.

Action: record start-to-start interval, complete loop duration, and read/planning/write/log durations separately. Measure crowded scenes and disconnects on the device. Skip/reduce optional planning work when its budget is exhausted; keep mechanism and drive updates timely. Evaluate hub bulk caching through the installed SDK/Pedro lifecycle so cache invalidation preserves fresh snapshots. Do not treat 0.2-second freshness limits as a desirable update rate.

### 7. Pickup utility is narrower than whole-match value — later optimization

`planning/pickup/PickupPlanner.java` optimizes expected tip points per second using injected load probabilities, confidence blending and cycle time. It does not represent remaining match time, partner assignments, other scoring objectives, or a measured probability that the entire collection route succeeds. Its geometric confidence blend is a heuristic, not a validated route success probability. The planner's preferred scoring poses also need to match the shots the runtime actually chooses from its range table.

Action: first populate measured collection and complete scoring-cycle costs. Later make the objective phase-aware and alliance-aware, and include pickup success, shot/tip outcome, recovery costs and time needed to park. Keep a simple driver-selected scoring fallback. Do not add more search depth until field logs show that target selection is the bottleneck.

### 8. The shot journal needs an outcome workflow — measurement gap

`scoring/ShotJournal.java` records commands, feedback and exits, and exposes `outcome(...)`; the competition runtime does not call it with observed scores/tips. An exit event establishes release, not successful scoring. Ordinary shots also turn RPM off unless deliberate pre-spin is requested, so recovery measurements need a consistent test protocol.

Action: export shot records with configuration revision and match/run identity, and join video or observer-labeled outcomes by shot ID. Compare hit rate, tip rate, cadence and faults across ball wear, battery condition and mechanism temperature. Use those measurements to choose pitch/RPM/gap and characterize safe recovery. Start with one gap per ball type. Enable moving shots or overlapping flights only when their measured envelope outperforms stationary cycles.

## December 5 development schedule

Dates below are proposed checkpoints, not evidence of completed work.

1. **October 11–18: lock interfaces and start building.** Finalize drivetrain, motor/servo allocation, ball feed path, sensors, electronics placement and service access. Order critical parts. Add basic driving/bench capabilities, the shot-request lifecycle repair and a wall-departure design. Get a driving chassis as early as possible. Keep one proven mechanical fallback rather than maintaining several unfinished shooter designs.
2. **October 19–November 1: complete first functional hardware.** Wire the chassis and prototype/assemble the launcher and feed system. Verify yaw neutral/stops, output feedback, ball containment and mixed-size feeding. Produce the first stationary shot data and a measured profile. Target a complete robot that can drive, intake and shoot by November 1.
3. **November 2–15: establish reliable scoring and a simple autonomous.** Measure both ball types near/mid/far and at intermediate distances. Implement preload shooting as repeated confirmed actions, a conservative fixed pickup only if dependable, and a tested park deadline. Test legal starting positions, late vision, jams, unknown inventory and safe cancellation. Use the measured outcome log.
4. **November 16–29: full matches and driver practice.** Run consecutive match-length rehearsals, practice human-player/alliance coordination and battery/service routines. Measure where time is lost. Optimize pre-spin and concurrent intake only after the hardware supports them. Change mechanisms only to solve a demonstrated limiting problem.
5. **November 30–December 4: stabilize the competition version.** Freeze major features, repair reproducible faults, verify backup/restore and configuration revisions, check spares and rehearse event setup. Prefer a dependable stationary shot and simple auto if moving/burst scoring remains unproven.

If the November 1 complete-robot checkpoint slips, reduce scope immediately: stationary shooting, reliable collection, a basic preload/leave/park autonomous and driver control. Preserve practice time.

## Proposed acceptance goals

These are initial team engineering targets, not FIRST requirements or Worlds benchmarks. Adjust them when real data establishes what matters.

- At least 30 complete feed/exit cycles per ball type and repeated mixed-ball transitions, with known inventory and no unintended extra releases. Count every jam and recovery.
- Stationary shot validation at independent intermediate distances and off-axis yaw. Start with a 95% intended-cell hit-rate target in the accepted envelope; record trial counts, spread and confidence bounds. Measure tipping separately.
- At least 20 autonomous rehearsals covering allowed starting configurations and perturbations. Initial goal: at least 19 reach the intended fallback/park result, with every failure explained and no unsafe release. A 19/20 observation alone does not establish a true 95% reliability rate.
- Ten consecutive full-match rehearsals without requiring an OpMode reinitialization, with consistent operator recovery and serviced batteries. Expand the test set after repairs.
- Log complete control-cycle timing; begin with a p99 interval goal of 50 ms or better and investigate every excursion toward the 200 ms fault boundary. Tighten the budget based on the measured yaw/launcher dynamics.
- Track p50/p95 request-to-exit time, collection-to-next-ready-shot time, actual tips per match, inventory faults, driver recovery time and park success. Compare improvements using repeated runs under similar conditions.

## After the first event: what could create a competitive advantage

Use match/video data to find the largest lost-time or lost-scoring contributor. Improve that contributor first: acquisition reliability, launcher spread, recovery cadence, route execution, or driver workload. Moving shots save time only if their hit/tip probability and recovery costs keep net cycle value high. Burst tipping needs separate physical evidence for impact point, hive state, arrival timing and rebounds. Dynamic pickup needs calibrated camera geometry and measured travel costs. Promote each feature behind a validated capability setting and retain the dependable fallback.

Qualification is also a team effort: the current advancement model combines qualification performance, alliance selection, playoff results and judged awards. Preserve evidence of the team's design decisions and measured control improvements for the portfolio. [Official advancement and Control Award sections](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU04.htm)

## Verification and reproductions

- `tools/run-desktop-tests.ps1`: **2,745 checks passed across 11 suites**. Counts: planner 99; piece tracker 180; state history 23; turret safety 72; Pedro mapping 45; safety regression 498; turret bench 114; audit regression 38; robot integration 691; moving shots 118; launcher calibration 867.
- `gradlew.bat :TeamCode:assembleDebug --offline --no-daemon`: **BUILD SUCCESSFUL**. Existing Java 8 target and Gradle deprecation warnings remain. APK packaging succeeded; no deployment or hardware testing was performed.
- The new `worlds-readiness.canvas.tsx` passed a strict standalone TypeScript 5.9.3 check against the installed canvas SDK. Existing unrelated canvases have type errors and were excluded from this check.
- `doc/audits/reproductions/WorldsReadinessProbe.java`: compiled against the desktop classes and reproduced the three failure paths below. The selector/recipes and 18-inch square are synthetic examples that isolate the current contracts, not measured robot settings.

Observed probe output:

```text
Late target: scoring remains IDLE; autonomous times out and parks without firing.
Next shot while WAIT_FOR_HIVE: start pulse is discarded; scorer later idles without firing.
Legal wall-contact rectangular start: FAULT / ROUTE / FOOTPRINT BLOCKED
```

To rerun after the desktop runner, compile the standalone probe with Java 8 target into `build/desktop-tests/classes`, using that directory as its classpath; run `WorldsReadinessProbe`. The probe asserts the current failures. When those failures are fixed, replace its failure expectations with meaningful regressions in the normal runner.
