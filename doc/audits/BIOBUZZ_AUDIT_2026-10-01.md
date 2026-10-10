# Dynamite BIOBUZZ code audit and robot development plan

Audit date: October 1, 2026. Reviewed commit: `20e25b3`. Hardware context supplied by the team: Wisconsin; four-motor mecanum; 300–330-degree belt turret; output absolute encoder; turret-mounted Limelight 3A; adjustable hood and two compression settings; stationary magazine with center transfer; fixed HuskyLens; dedicated odometry planned. First qualifier: early December. This is an audit and engineering recommendation, not a hardware certification or measured performance rating.

**Assessment**

The architecture is useful and worth preserving. Hardware-independent controllers, immutable RobotState, historical pose interpolation, explicit invalid states, injected travel/shot models, and refreshed commitments are good foundations. The current implementation is a prototype software stack. It has no competition autonomous OpMode, no complete competition TeleOp, no physical TurretIO implementation, no shooter controller, no sensed inventory, and no HIVE state estimator. Automatic unwind exists as a calculated request; the physical drivetrain does not consume it.

The most valuable next improvement is to close the complete intake → identify → stage → aim → release → confirm exit → confirm tip → reacquire loop on a physical robot. Additional planner sophistication should follow measured scoring data.

**Verification performed**

- Existing desktop suites: 99 planner + 180 tracker + 23 history + 72 turret checks = 374 passed.
- Existing PedroMappingScenarios, compiled separately with the cached Pedro 3.0.1 core: 45 passed. Total existing checks run: 419.
- Offline Android `:TeamCode:assembleDebug`: successful, using the existing user Gradle cache and Android Studio JDK. Deprecation/Java 8 compiler warnings did not prevent packaging.
- Additional isolated desktop probes reproduced the behaviors below against unchanged production code. Probe source and observed output are retained in `doc/audits/reproductions`; compiled output is under the ignored `build/audit` directory. They are diagnostic examples, not hardware tests.
- No robot was connected or powered. No CAD was available for inspection. Production Java and configuration were not changed.

The desktop runner omits the Pedro suite, SDK-dependent vision, hardware OpModes, and actual mechanism dynamics. Passing its checks does not establish full match readiness.

**Confirmed defects and integration hazards, in priority order**

P1 means fix before this code controls competition hardware. P2 means fix before enabling the relevant automation or tuning that feature. Deliberate placeholders and missing systems are listed separately.

| Priority | Finding | Evidence and match consequence | Recommended correction |
|---|---|---|---|
| P1 | A refused manual drive request retains the previous movement | `pedro/PedroManualDrive.java:38–46` returns false without clearing the follower. `test/PedroIntegrationTestOpMode.java:74–76` still calls update. Probe: request forward 0.5, invalidate heading, request zero; return false, actual output still forward 0.5. | On refusal with an existing follower, explicitly transition to stop/zero and ensure the next update applies it. Test valid motion → invalid heading/input → zero output, and recovery. |
| P1 | Rear dead-zone crossing flips the chosen side | `turret/TurretController.java:129–141` selects the boundary from wrapped ideal bearing without preserving the existing side. With measured turret +150°, bearing 179° → 181° changes desired +160° → −160°, power +0.35 → −1.0, chassis request +1.047 → −1.047 rad/s. A two-degree target change creates a 320-degree command change. | Latch the chosen dead-zone/unwind side until the target is reachable again, accounting for measured position and chassis motion. Retain the prohibition on crossing the physical stop. Exercise noisy bearings around ±180° and both approach directions. |
| P1 for planner execution | Planner issues SHOOT_NOW for an empty robot | `planning/pickup/PickupPlanner.java:221,299–312,351–364`: zero-length route is always SHOOT_NOW, including zero inventory and zero expected reward. Probe reproduced it. Missing MapTipModel entries can also make every candidate worth zero. | Add explicit WAIT/SEARCH/NO_FEASIBLE_ACTION or equivalent execution guard. Do not feed without a confirmed staged piece. Distinguish unknown model data from a measured zero probability. |
| P1 for autonomous pickup | Selected capture paths need not fit the field | `planning/pickup/CaptureGeometry.java:33–36` places the capture point six inches beyond the ball. `EuclideanTravelTimeModel.java:52–57` only prices straight-line distance/yaw. Probe: ball x=143, robot x=120 → selected capture x=149 in a nominal 144-inch field. Body clearance fails sooner. | Check the swept robot/intake footprint, field walls, HIVE legs, FLOWERS, and approach feasibility. Generate wall-specific approach geometry. Enforce hard feasibility separately from timing estimates. |
| P2 | Automatic unwind never completes by itself in a stationary-target case | `turret/TurretController.java:149–176` releases the latch at 130°, but requests no yaw at or below 140°. Ideal chassis simulation converged to 140° with unwindActive=true and zero omega after 200 s. | While latched, drive toward an interior goal at/below the exit threshold, with a smooth bounded request. Test that it actually releases. |
| P2 | Robot pose validity is not freshness or device health | `pedro/PedroMappedReading.java:54–62` checks finite numbers; `RobotStateSource` has no acquisition timestamp/status. `PickupPlanner.java:77–79` ignores RobotState age. Probe accepted a ten-second-old robot state with current observations. | Carry acquisition time, localizer health, and reset generation. Establish freshness thresholds from measured sensor/loop timing. Disable automatic firing/pickup when state is unreliable; retain a deliberate manual driving fallback. |
| P2 | Repeated observations artificially mature tracks | `vision/pieces/PieceTracker.java:342–354` increments hits for every association, even with the same observation timestamp. Probe processing one observation five times changed confidence 0.20 → 1.00. The camera adapter stamps requests, not unique frame captures. | Track distinct acquisition/frame events where supported. Otherwise bound poll rate and confidence growth by elapsed camera time; do not treat polls as independent evidence. Reject out-of-order updates. |
| P2 integration gap | Collected pieces and localization resets lack tracker lifecycle operations | PieceTracker has no consume/remove/reset API. After a detection disappeared, the probe still published its old track at t+0.1 s, within the intended coasting window. Removing it from one returned list cannot remove the internal track. | Add consumed-track suppression and a tracker reset/rebase policy. On localization reset clear/rebuild tracks, history, commitments, and controller baselines together. Test collection followed by replan and pose discontinuity. |
| P2 when D/feedforward are enabled | Turret feedback and limit handling are incomplete for a real mechanism | `TurretController.java:63–87` differentiates error, ignores measured velocity in control, and feeds the unclamped ideal target rate even if desired angle is clamped. Invalid-input holds preserve derivative memory. `blockPowerIntoStop:179–187` acts only at a physical boundary. | Add a bounded position/velocity reference, measured velocity feedback, acceleration/braking constraints, voltage-aware characterization, and a clean derivative reset after faults/target switches. Feed forward the feasible reference rate. Measure stopping distance before selecting soft-limit margin. |
| P2 gameplay limitation | Tip probability depends only on onboard counts | `planning/pickup/TipModel.java:14`; `BallLoad.java:3–5` deliberately discards queue order. This cannot represent current HIVE contents/state, distance, impact speed, spin, shot cadence, partner contributions, or which CELL is up. | Evaluate a ShotPlan using HIVE estimate, queue sequence, shot geometry and measured outcome data. Score marginal expected progress/points and recovery, not just isolated probability that a load produces one tip. |
| P2 gameplay limitation | Shot timing and shot execution are disconnected | `PreferredPoseShotSetup.java:81–102` returns a minimum time but not the selected shot pose/recipe; the load argument is ignored. PickupPlan endpoint is the last capture pose, not the chosen firing pose. | Return the chosen pose/shot recipe or share one deterministic selection method with the executor. Include ball-dependent staging, compression changes, spin-up, per-shot recovery and HIVE settle/reacquisition time. |
| P2 runtime concern | Planning and tracking costs have no Control Hub timing budget | Planner explores up to 5,861 routes and allocates per route. Tracker uses exhaustive association and has no independent hard track cap. HuskyLens reads occur synchronously in an OpMode example. | Measure loop p50/p95/p99/max under crowded scenes. Run fast mechanism control continuously and bounded planning on a measured lower cadence or events. Cap tracker work; service vision without allowing a slow I2C transaction to monopolize control. |

Source paths in the table are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. These findings describe the current code. The empty-shot, footprint, and health findings do not claim that an existing competition autonomous already executes those unsafe decisions—there is no such autonomous yet.

**Incomplete capabilities that must be explicit before integration**

1. `turret/TurretIO.java` is an interface; its only implementation is simulated. Add real angle/velocity acquisition, sign/zero calibration, output plausibility, motor shutdown, and fault recovery.
2. `pedro/PedroFactory.java:150–164` sets all feedback/braking models to zero and uses 1 in/s stand-ins. Motor directions, odometry geometry, and encoder scales in PedroConstants remain placeholders. Constructing a follower is not tuning it.
3. `test/HuskyLensPieceTestOpMode.java:32,43,68–69` has empty pose history and unconfigured projection, so it cannot produce field pickup targets. Build a combined odometry/vision calibration and validation OpMode.
4. There is no Limelight adapter, camera/turret history, shot solution, RPM controller, staged-piece sensor logic, jam handling, or HIVE transition detector.
5. The demo tip rates (including 90% for three NECTAR) are clearly labeled fixtures. They must never be used as measured team performance.

**Before turret CAD is released on October 2**

These are engineering recommendations based on your design, not measured properties of your CAD.

- Preserve output-shaft absolute sensing. Place its electrical wrap in the inaccessible rear sector so normal motion has an unambiguous signed angle. Calibrate sensor zero against a physical datum; reboot at several turret positions. A single-turn absolute reading does not by itself validate the belt or wiring.
- Verify the actual am-5200 variant, pinout, voltage range and signal path on a bench before designing the harness. AndyMark lists quadrature, absolute PWM and absolute analog for its FTC non-CAN version. Incremental quadrature alone does not provide absolute startup angle. Use a verified supported acquisition path, and check angle with an independent reference. [AndyMark specifications](https://andymark.com/products/hex-bore-encoder), [REV analog inputs](https://docs.revrobotics.com/duo-control/sensors/analog).
- Specify physical stops, output travel, cable travel, and operating limits independently. Include the Limelight USB cable in the full sweep. A five-degree software margin is a placeholder; derive the real margin from speed, inertia, braking and loop delay.
- Check the swept shooter/hood/camera envelope at every turret angle and hood/compression setting. R102 requires an 18-inch starting cube; R105 requires physical compliance with the expanded 18×24×29-inch volume. Software clamps alone cannot satisfy that construction requirement. [Construction rules](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-12).
- Put sensors at the magazine entry, transfer/staging throat and shooter exit where practical. Inventory must count every controlled piece across intake, transfer, magazine and feeder. A full four-piece magazine plus one staged piece is five. Use a capacity interlock with in-flight reservations and physical overflow prevention. G407 allows at most four controlled scoring elements. [Game rules](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-11).
- Keep the floating intake bounded with positive travel stops and serviceable pivots. Test mixed piece sizes, side-by-side contact, wall pickup, and pickup while turning. A floating roller alone does not guarantee the transfer throat can handle both sizes.
- Make compression hard stops adjustable during R&D, then lock them positively. Support the cam so flywheel loading cannot move its setting. Provide access to clear a stuck ball without disassembling the turret.
- Allocate every actuator now. Four drive + turret + flywheel + intake + transfer already uses eight motors. R503 limits the robot to eight motors and eight servos. A separately motorized spin roller requires a revised motor allocation. An upper roller mechanically coupled to the flywheel can test a fixed spin ratio; independent spin control has additional cost. [Construction rules](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-12).
- Prototype the center-transfer interface at multiple turret angles. Test whether a staged ball can bridge or drag across the stationary/rotating boundary. Use an escapement that releases exactly one ball and can reverse/eject. Design feed blockage to be observable.

**Two camera transformations deserve special attention**

A Limelight rigidly mounted to the turret is moving relative to the chassis. Store turret angle alongside chassis pose at the camera exposure time. RobotStateHistory currently stores only chassis motion. Reconstruct camera pose from chassis → yaw pivot → turret → camera, using measured offsets. Camera movement during image latency creates aim error even while the chassis is stationary.

Also, BIOBUZZ's AprilTag clusters are attached to the moving CELLS. A fixed field-map pose for one HIVE state cannot be assumed correct during a tip or in the other stable state. Do not inject these observations into global odometry as if they were fixed field landmarks. Use them to estimate the CELL/HIVE relative state, or use an explicitly state-aware map/transform with rejection during transitions. This is an engineering inference from the moving-tag geometry in the [ARENA manual](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09).

Limelight's FTC guide describes camera extrinsic configuration, target/camera transforms, distinct FTC-centered botpose coordinates, and result staleness. Your Pedro frame is different from that centered frame. Convert units/axes once in an adapter; never feed raw botpose directly into RobotState. If using MegaTag2 later, its orientation and extrinsics must describe the same reference frame consistently. For V1, camera-relative goal measurement plus dedicated odometry is the cleaner starting point. [Limelight FTC guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

For fine aim, calibrate the target point and camera-to-barrel offset at multiple ranges. Centering a tag is not automatically centering the upward CELL opening. Reject wrong-alliance/wrong-CELL detections, stale frames and inconsistent poses. Keep the camera rigid to the yaw assembly if possible rather than allowing hood adjustments to change its extrinsics.

**Recommended competition software behavior**

Keep the current subsystem separation. Add one robot coordinator that reads hardware into one timed snapshot and grants each subsystem clear actuator ownership. Avoid separate subsystems writing competing drivetrain or feeder commands.

The shot sequence should be nonblocking:

`SPIN_UP → AIM_AND_STAGE → READY → FEED_ONE → CONFIRM_EXIT → WAIT_FOR_HIVE → REACQUIRE`

Every state needs a timeout and a bounded recovery route. Jam recovery should stop feed, reverse the appropriate transfer, retry a limited number of times, and expose a clear fault to the driver. A feeder command is not evidence that a ball left the robot.

A READY gate should require a confirmed staged piece/type, correct compression and settled hood, stable flywheel speed, valid/reachable aim, fresh target estimate, acceptable chassis/turret motion, and no HIVE transition or mechanism fault. Tracking=true means calculations were possible; it does not mean ready to fire. Display the reason a shot is inhibited and give drivers a predictable cancellation/manual recovery action.

Make chassis unwind a deliberate drive-assist policy. TurretCommand uses rad/s, but PedroManualDrive's third input is turn power. Never directly add those values. Convert through a measured yaw controller/model with saturation and driver arbitration. Chassis heading should be planned so intake approaches do not unnecessarily force the turret into the rear sector.

For motion compensation, first measure shots while stationary. Current bearing-rate feedforward holds geometric aim; KV=0 currently disables its motor contribution. It does not solve projectile motion from a moving muzzle. Later compensation must consider release delay, muzzle position/velocity (including offset rotation), time of flight and shot calibration. Enable it only when it beats stop-and-shoot in completed tips per match.

Use one monotonic timebase for sensor/pose/turret history. Reset all dependent state coherently after a localization correction or OpMode clock reset. Carry observed inventory as an ordered queue with ball type/identity confidence, stage location and acquisition/exit events; reserve onboard counts as an aggregate view for planning.

**Gameplay ideas with the greatest likely return**

These are hypotheses to validate on your robot.

1. **Measure complete tip cycles.** Your 6–8 s target must include pickup, transfer, staging, firing all necessary balls, tip settlement, target change and being ready to start the next cycle. Measure median and slow-tail times, failed-tip recovery, and human loading delays.
2. **Recognize HIVE state before each release.** Fuse tag geometry, visible CELL orientation and observed motion into UNKNOWN / SIDE_A_UP / TIPPING / SIDE_B_UP. Stop feeding during transition and confirm the stable destination before resuming. The scoring manual warns that continued launch into a tipping HIVE can prevent a completed tip. [Scoring criteria](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-10).
3. **Exploit progress already in the CELL.** A near-threshold CELL may need one more piece rather than a fresh full load. Track your releases and tip outcomes, include partner contributions with uncertainty, and reacquire after any ambiguous event. Do not assume onboard load alone determines tip probability.
4. **Test impact location and cadence, not merely ball count.** For three-NECTAR/four-POLLEN experiments, sweep hood/RPM/compression first, then landing position, shot spacing and spin. Seek a broad repeatable operating region. A single successful high-energy tip or a sensitive narrow setting is not a match recipe.
5. **Keep optional spin control conditional.** Backspin can change flight, landing and retention; its net benefit here is empirical. Add the upper roller only when a controlled comparison shows higher tip reliability or shorter recovery-adjusted cycles than the simpler shooter.
6. **Overlap preparation with travel.** Identify the next queued piece, set compression/hood, pre-spin and pre-aim while moving when mechanically compatible. Switch settings between pieces only with a confirmed clear throat. Overlap measured independent tasks rather than assigning a fixed 0.8 s shot delay to every load.
7. **Assist drivers with one repeatable cycle command.** Automatic recipe selection, aim readiness, inventory feedback and jam recovery should reduce driver workload. Give drivers control of route/position and an obvious abort. Add automatic pickup approach only after its footprint and vision failures are tested.
8. **Design for floor reacquisition.** Use the wide intake to recover dispersed pieces after they land and pick approaches that preserve turret accessibility. G409 restricts catching/deflecting pieces directly from a tipped HIVE, and G408 prohibits controlling opponent NECTAR. Capacity and color rejection need to work at the intake/transfer as well as in the planner. [Game rules](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-11).
9. **Keep a FLOWER option open without committing to a lift.** A repeatable close-range low-energy shot or short servo-controlled chute might add late-match value if mechanically feasible. Test it on a prototype. The opening is about four inches across and 21.5 inches high, so this is a precision problem. [Field dimensions](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09). Ownership belongs to the alliance with the top-most scoring NECTAR, so reserving one can matter; compare that opportunity with an additional tip and coordinate with your partner. [Scoring criteria](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-10).
10. **Make parking and partner coordination reliable.** Use conservative autonomous fallback and a park deadline based on measured worst-case return time. Agree on which robot handles each shooting/retrieval lane and FLOWER task. Avoid duplicate pursuit and firing through one another's routes.

The current October 1 season page lists TU03, including updated game details/rules. [Official season materials](https://ftc-resources.firstinspires.org/ftc/archive/2027/game). HIVE tips are 20 points; local-event POLLINATOR thresholds are four and seven tips, with championship thresholds still TBA. Treat RP targets as alliance goals. [Point values and thresholds](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-10).

**A realistic autonomous progression**

Build a dependable preload routine and leave/park fallback first. Then add one fixed pickup cycle, then sensor-confirmed retry/reacquisition, then multiple cycles, and finally dynamic route selection. Do not put a general vision planner ahead of a reliable fixed-route baseline.

For the 30-second AUTO, four six-second cycles consume 24 seconds before any time not included in those cycles. Four eight-second cycles already exceed AUTO. Three/four tips are stretch goals until timed full routines demonstrate enough setup/parking margin.

Reliability compounds: if each tip cycle independently succeeds 90% of the time, four successful cycles occur only about 66% of the time; at 98%, about 92%. Independence is illustrative—shared faults can make real results worse. Record complete routine success as well as individual shot success.

**Evidence targets for a robot you can trust**

Suggested engineering acceptance gates, not official qualification standards:

| System | Initial gate | What to record |
|---|---|---|
| Turret | 100 sweeps and rear-sector transitions without stop impact, cable snag, branch flip or loss of position; reboot correctly at several angles | Angle error, speed, current, voltage, timing, fault state |
| Intake/transfer | At least 100 varied pickup/stage attempts with both piece types and mixed queues; measure failures and confirm no fifth-piece intake | Intake speed, approach, queue state, jams and recovery duration |
| Shooter | Screening grid followed by at least 100 independent trials per shortlisted tip recipe, split across representative conditions | Actual pieces, HIVE starting state, battery, hood/compression, RPM before/after each shot, cadence, landing/retention, tip result |
| Vision/localization | Compare measured floor points and aim at several ranges, headings and turret rates; intentionally test stale frames and occlusion | Position/angle residuals, acquisition age, rejected observations, reset behavior |
| Autonomous | 20 consecutive complete routine runs with bounded fallback/park behavior, followed by varied layouts and obstruction cases | Completed tips, total time, pose drift, recovery, park |
| Match operation | 10 consecutive full simulated matches without an unhandled software fault or manual disassembly | Points/tips, cycle distribution, fault count, driver interventions |

100/100 independent successes only puts the one-sided 95% lower confidence bound near 97%; 20/20 full routines is an initial gate, not proof of 99% reliability. Use varied balls, orientations, battery states, both HIVE directions, and realistic field assembly variation.

Log robot/target/turret state, inventory events, shot recipe and measured RPM, camera age, controller output, HIVE state/confidence, loop timing and outcome. A timestamped event log plus matching video makes tuning and failure analysis much faster than telemetry alone.

**Schedule to the early-December qualifier**

| Window | Deliverable | Exit condition |
|---|---|---|
| Oct 2–11 | Turret/shooter bench rig, encoder/harness validation, mixed-piece transfer prototype; repair drive refusal and turret branch/unwind behaviors | Measured limits and safe startup/shutdown; shots and staging observable |
| Oct 12–25 | Working drivetrain/intake/magazine; real odometry tune; integrated manual scoring; recipe screening | Repeatable stationary scoring and recoverable jams; inventory verified |
| Oct 26–Nov 8 | READY/FEED/EXIT/HIVE state flow; fixed-route AUTO; autonomous park fallback; combined camera calibration | First full timed autonomous and full simulated matches |
| Nov 9–22 | Cycle reduction, varied-condition recipe validation, driver practice, partner simulation | Repeated routine success and quantified slow-tail cycle/recovery times |
| Final 10–14 days before qualifier | Freeze major mechanics; regression runs, spares, pit procedure, portfolio and judging practice | Proven configuration; changes only for demonstrated failures |

If mechanisms are late, reduce autonomous ambition before reducing test/practice time. Finish hardware early enough to gather data; a highly adjustable V1 should become a documented, locked competition configuration.

**Getting to FIRST Championship**

A robot alone does not guarantee advancement. Section 4 combines qualification performance, alliance selection, playoffs and judged awards; Inspire first is worth 60 advancement points and a winning alliance earns 40 playoff advancement points. [Advancement manual](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-04).

For Dynamite, document this R&D as student-owned engineering: competing hypotheses, failure data, CAD changes, measured performance and how students made decisions. Maintain outreach/team development and judging preparation alongside the robot. I could not verify a current BIOBUZZ Wisconsin Championship slot allocation from the accessible official material; do not reuse last season's allocation as this season's requirement.

**Recommended implementation order**

Repair fail-stop behavior and turret rear-sector/unwind logic → instrument the physical transfer/shooter/turret → implement ordered sensed inventory and firing gates → tune drivetrain/localization → add HIVE-aware shot planning and fixed-route autonomous → add footprint-safe dynamic pickup → evaluate motion shots, spin roller and FLOWER option from measured match benefit.

The likely competitive advantage is a fast, repeatable tip cycle with immediate fault recovery and a practiced drive team. The existing planner can support that, but the mechanism data and complete executor must come first.
