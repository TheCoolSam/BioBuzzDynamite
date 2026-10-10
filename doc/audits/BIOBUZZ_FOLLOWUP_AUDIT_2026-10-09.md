**BIOBUZZ code audit — October 9, 2026**

This report records the behavior before repairs. For subsequent code changes,
verification, and remaining physical dependencies, see
[the implementation status](BIOBUZZ_IMPLEMENTATION_2026-10-09.md).

Reviewed the working tree based on `7ffbe3f`, including the uncommitted turret bench implementation. This is a software readiness review. The code builds, and its existing regression checks pass, but it is still a collection of subsystem prototypes and test OpModes rather than a complete competition robot program.

**Verification and scope**

- `tools/run-desktop-tests.ps1`: **1,031 checks passed** across seven suites: planner 99, tracker 180, state history 23, turret safety 72, Pedro mapping 45, safety regressions 498, turret bench 114.
- Offline `:TeamCode:assembleDebug`: **BUILD SUCCESSFUL**, including SDK-dependent Java compilation and APK packaging. Output: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
- Additional desktop probes reproduced the findings below. Source: `doc/audits/reproductions/FollowupAuditProbe.java`; output: `doc/audits/reproductions/FOLLOWUP_OBSERVED_OUTPUT_2026-10-09.txt`. These print current behavior and are diagnostic probes, not passing regression assertions for the desired behavior.
- Inspected team Java, build/test scripts, and the Fusion helper entrypoint and validation scripts. Python AST syntax checking reported 15 files parsed; an empty-folder probe reproduced the mesh-validator issue. Fusion operations were not executed.
- The existing working-tree diff passes `git diff --check`. Build warnings cover Java 8 compilation under the installed JDK 25, deprecated SDK/Gradle APIs, and native debug-symbol stripping. They did not prevent packaging.
- No hardware was connected, powered, deployed to, or tested. Existing user changes were preserved. Added this report and reproduction evidence; robot runtime code was not changed. SDK samples were not treated as team competition implementations.

**Progress since the October 1 audit**

The five repaired areas are covered by passing regressions: refused drive demands replace previous movement with zero; rear-sector turret recovery retains its side across 179/181 degrees; simulated unwind completes on both sides; empty or zero-value loads produce WAIT; and out-of-field capture endpoints are rejected or replaced by an alternative approach. The last item establishes endpoint bounds, not full robot clearance.

Real REV turret bench I/O now exists. It has arming, deadman, latched emergency stop, calibration/readiness gates, output caps, directional limits, sensor plausibility checks, and stopping telemetry. Invalid controller timing/data also resets controller history. The old audit's claim that no physical TurretIO implementation exists is historical and no longer describes this working tree.

**Confirmed defects to repair before automatic pickup/scoring**

P2 here means repair before enabling the affected automation. No competition executor currently consumes these plans, so these are demonstrated planning/tracking defects, not evidence that an existing match OpMode is executing them.

| Priority | Finding and evidence | Required change |
|---|---|---|
| P2 | **Invalid time models remain executable plans.** `planning/pickup/PickupPlanner.java:336–354` substitutes 1,000,000 seconds for invalid shot/travel estimates. A positive tip probability still yields positive utility. Probe: NaN travel produces PICKUP at 1,000,000.8 s; NaN shot setup produces SHOOT_NOW at 1,000,000 s. | Represent unavailable/infeasible estimates explicitly, reject the affected route, and return WAIT when no executable alternative exists. Preserve a non-null WAIT baseline when rejecting the zero-pickup route. Cover null/missing, NaN, infinity, and negative estimates; ensure shot poses with no valid solution also fail closed. |
| P2 | **Robot-state freshness is unchecked.** `PickupPlanner.java:91` checks pose validity and current time but never the robot snapshot's timestamp. `turret/TurretStateAdapter.java:30` also drops the timestamp. `pedro/PedroMappedReading.java:54` treats finite values as valid, without localizer acquisition age/health. Probe: a 10-second-old pose produces PICKUP with fresh pieces. | Carry acquisition age, device health, and reset generation; require measured freshness thresholds for automated decisions. A freshly stamped estimator snapshot alone cannot prove that the localizer delivered new data. |
| P2 | **Duplicate and older observations corrupt track evidence.** `vision/pieces/PieceTracker.java:342–355` increments hits on every match and overwrites lastSeen without an ordering check. Probe: five copies of one timestamp raise confidence 0.20 → 1.00; an older frame rolls lastSeen 0.20 → 0.10 s. `HuskyLensPieceObservationSource.java:94` assigns caller polling time to blocks rather than a unique acquisition/frame identity. | Reject older events and prevent duplicate evidence from maturing a track. Where the camera cannot expose frame identity, bound confidence growth by measured acquisition cadence and expose timing uncertainty. Validate stale/out-of-order data before association. |
| P2 | **Collected tracks and localization resets have no lifecycle API.** PieceTracker owns private tracks with no consume/remove/clear/rebase operation. Probe: after collection is assumed and detections disappear, the internal track remains published within the coasting window. Normal coasting is useful; collection needs a different event. | Add confirmed consumption and reset operations, with a policy preventing delayed frames from resurrecting a consumed piece. On localization reset clear/rebuild tracker, pose history, planner commitment, and controller baselines together. |
| P2 | **Mesh validation can succeed without checking a mesh.** `tools/fusion/BioBuzzHoodTools/validate_print_meshes.py:10` uses nonrecursive `glob('*.stl')`; an empty directory exits successfully with `[]` and writes mesh_validation.json. Important checks at lines 13 and 45 use Python assert and disappear under optimized execution. | Require a nonempty expected file set or manifest, handle nested fit-print directories explicitly, and use explicit validation exceptions/exit status. Mesh topology alone does not establish units, physical fit, or mechanism clearance. |

All Java locations in the table are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

**Integration blockers and unfinished capabilities**

1. **Build the robot coordinator and competition OpModes.** The six annotated team OpModes are diagnostics/simulations; there is no team `@Autonomous`. Add one owner of timed hardware reads and actuator writes, an integrated TeleOp with cancellation/manual recovery, and a fixed-route autonomous with deadlines, timeout recovery, and a park fallback. The planner currently chooses strategy and never drives or feeds.

2. **Configure and characterize drivetrain/localization.** `pedro/PedroConstants.java:35–73` contains all-FORWARD motor directions, zero pod offsets, unit encoder scales, and 1 in/s limits. `PedroFactory.java:152–164` sets feedback/braking controllers and models to zero. Confirm hardware names, motor/pod signs, odometry device and geometry, scale, and Hub orientation where relevant; measure velocity/braking and tune path following. The Pedro test is active and can request stick power after device mapping succeeds, so telemetry that calls values placeholders is not a motion interlock.

3. **Complete turret sensing and bench evidence.** `turret/TurretHardwareConfig.java:6–23` has UNSELECTED acquisition, NaN zero/analog endpoints, and false readiness flags. Consequently the current bench cannot enable motion. Select the verified absolute signal path and fill calibration from measurements; PWM capture remains unimplemented (`RevTurretIO.java:40–42`). Validate startup at several physical angles, signs, fault detection, limits/cables, stopping distance, and rearming. Plausible/stuck analog voltages cannot establish sensor health by themselves.

4. **Provide the competition turret integration.** `RevTurretIO.java:89` deliberately makes generic setMotorPower calls unarmed, so simply replacing simulated I/O with this class will command zero. Define a guarded competition command/lifecycle interface without bypassing the bench's protections. Connect chassis unwind through a measured yaw controller and driver arbitration: turret requests rad/s, while Pedro manual drive consumes turn power. Before enabling nonzero KD/KV, add a feasible position/velocity reference and velocity-aware feedback; the current controller differentiates error and computes feedforward from the ideal bearing even when the position reference is clamped. Gains/physical margins are unmeasured.

5. **Implement the complete scoring mechanism.** There is no runtime flywheel RPM controller, hood/compression controller, sensed intake/transfer/staging/exit inventory, feeder executor, jam recovery, or HIVE state estimator. BallLoad is an aggregate count, not a sensed queue. Implement ordered piece inventory and in-flight capacity accounting; stage and exit confirmation; ball-specific recipes; settled hood/compression and stable RPM checks; target/aim readiness; one-piece feeding; bounded recovery; and tip/target reacquisition. Every state needs a timeout and driver-visible inhibit/fault reason.

6. **Calibrate and connect vision.** `test/HuskyLensPieceTestOpMode.java:32,43` uses empty history and unconfigured projection, so it cannot publish usable field pickup targets. Combine real odometry history with measured pixel-to-floor calibration, camera IDs/alliance configuration, and measured latency. Add the missing Limelight targeting adapter, exposure-time turret/chassis history, camera-to-barrel transforms, and target identity/freshness validation. RobotStateHistory currently stores chassis state only.

7. **Finish motion feasibility and shot-plan contracts.** `PickupPlanner.java:67–69` defaults to zero robot clearance. Probe: the default model selects a robot center just one inch from the right wall. `FieldBoundsCaptureFeasibility.java:39–46` checks center endpoints, not rotating body/intake sweep or field obstacles. `CaptureGeometry.java:47` tries eight final approach directions without providing a staging segment that executes that approach. Inject measured footprint and collision/path feasibility. `PreferredPoseShotSetup.java:81–102` returns a scalar time while discarding the chosen scoring pose and ignoring the load; return an executable shot pose/recipe and price the full feed/recovery/tip cycle. Distinguish unknown tip-model data from measured zero probability and replace demo timing/probability fixtures with measured data.

8. **Establish runtime and deployment evidence.** Tracker assignment is exhaustive (`PieceTracker.java:188–281`) and has no independent track/work cap; the planner can score thousands of routes; HuskyLens acquisition is synchronous. Measure Control Hub loop timing under crowded scenes and faults, cap work, and schedule planning/vision so mechanism service stays timely. Add integrated event logging and hardware acceptance runs. RevTurretIO transactions, gamepad latency, actual stopping, localizer faults, sensor disconnects, and complete scoring/autonomous behavior are outside desktop test coverage. `TeamCode/TESTING.md:16` is stale about Pedro being excluded: the current runner includes mapping and simulated follower regressions, but still excludes hardware PedroFactory/PedroRobotStateSource integration.

**Suggested implementation order**

1. Repair invalid-model planning, state freshness, track evidence/lifecycle, and validator failure reporting; turn the probes into meaningful regressions when those fixes are implemented.
2. Configure/tune drivetrain and establish safe physical turret acquisition/bench behavior.
3. Complete sensed inventory and manual stationary scoring with firing gates and bounded jam recovery.
4. Integrate calibrated target vision and chassis unwind; deliver a dependable preload/fixed pickup autonomous with park fallback.
5. Add measured footprint-safe dynamic pickup, then optimize full cycle times using robot data.

The CAD helper is development tooling, not shooter-control software. The latest `tools/fusion/BioBuzzHoodTools/prints/2026-10-08/metal_mount/PRINT_AND_BUILD.md:90` still lists horn/link pin retention, physical compression stops/shaft retention, the 80T gear's rail attachment, and loaded sliding/servo tests as unfinished. These are existing documented mechanical blockers; this audit did not independently certify the CAD or print files.
