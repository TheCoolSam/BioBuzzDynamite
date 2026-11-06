# Ragnar Mark 4: design requirements and current code plan

Reviewed October 11, 2026. This is a proposed design baseline and a read-only review of the current repository. No robot Java or Fusion geometry was changed. Mark 3 remains the backup. The saved Mark 4 sideways selector is a rejected concept, not the design specified here or a print-ready assembly.

## Design baseline

Optimize repeatable release, reliable feeding, fast recovery and serviceability within the robot envelope. Minimum size is not the objective. The complete starting robot must fit the team's 18 x 18 x 18 inch requirement, including chassis, intake, feed path, camera and wiring. Check both the starting envelope and collisions throughout operating yaw/pitch/compression travel.

- **Short contact shoe:** substantially reduce Mark 3's contact wrap. Keep enough capture and acceleration length to avoid slipping; determine the final length and release edge from physical tests rather than estimating AiCitizens' geometry from an oblique screenshot. Use interchangeable contact surfaces to compare a fixed shoe with passive rollers. Shorter wrap does not by itself guarantee accuracy or eliminate spin.
- **Independent geared pitch:** a lightweight bearing-supported carrier rotates around the flywheel axis as the initial layout candidate. The gear segment, guides, carriage and compression servo follow that carrier. The compression shoe moves inside it; compression adjustment must not move the pitch gear mesh. Independently support the carrier so spinning the flywheel cannot drag it around. Synchronize both sides structurally or by a cross-shaft; check deflection and backlash at the actual shoe.
- **Compact dual-guide carriage:** directly support the shoe from a stiff crossmember between the guides. Avoid the previous long cantilevered printed eye/bracket. Use bushings or bearings and accessible shaft retention. Guide OD, straightness and surface finish must be measured; an M4-threaded standoff's name does not establish its outside diameter or suitability as a sliding shaft. Four-bar links remain an alternative if they outperform this assembly under load.
- **Two compression settings initially:** one calibrated gap for pollen, one for nectar. Their nominal diameters differ by 20.32 mm, but the required gap difference depends on the chosen compression for each ball. Model the actual gap along the whole contact region in both positions; translating a curved arc does not preserve concentric clearance everywhere. Preserve adjustment around both settings for testing. Provide retention and protection against overtravel; servo position alone is not a mechanical load rating.
- **Existing wheel stack:** two 72 mm Hogbacks beside the 60 mm steel inertia disc on one shaft. Retain rigid shaft support and secure hubs. These are one contact stack, not two opposing powered wheels. Verify actual motor SKU/RPM before selecting the drive ratio. Keep motor mass on the yaw frame where possible and make the belt/direct-drive choice from measured speed and available packaging.
- **Passive roller option:** no extra DC motor is allocated. Passive rollers must have low bearing resistance and must release each ball consistently; they do not independently command spin. A powered roller can alternatively share the main flywheel motor through a transmission, at the cost of drive complexity and a fixed surface-speed ratio. It is not required for the baseline.
- **Centered, repeatable feed:** entry guides must accept both ball diameters without an abrupt ledge at the moving shoe. Use shallow, symmetric centering features or side guides with adequate nectar clearance. A wide concave surface can create different contact patterns for the two sizes, so verify both. The stage position must stay outside the moving compression region while changing ball settings.
- **Full pitch feed check:** rotating the short shoe around a fixed wheel changes the capture region relative to a fixed feeder. Check actual ball capture at every intended pitch, not only empty-part clearance. If this cannot be maintained, reconsider the feed transition or pitch the shooting module; do not assume shortening the hood solves feeding.
- **Yaw and sensing:** use the AndyMark Robits 5.5 inch ID turntable, Axon yaw drive and a physically supported output-side absolute encoder. Encoder housing must be restrained; its bore must turn with the measured output through a verified coupling/ratio. Mount the Limelight rigidly on the yaw base, compact right-side position, independently of hood pitch. Verify line of sight and cable motion across all useful states.
- **Strength and manufacture:** structural metal where practical, short supported printed load paths, serviceable contact shoe and shaft access, repeatable print orientation. New M4 clearance holes are 4.2 mm; retain 5.8 mm insert pilots where the selected insert calls for them. Purchased hardware interfaces keep their actual vendor dimensions. Torque assessment must include gravity, acceleration, friction and ball reaction at the operating supply voltage; stall torque is not continuous holding capability.

## Motor allocation

The present code maps four drive motors plus flywheel, intake, transfer, feeder and DC yaw: nine DC motors. The proposed Axon yaw changes the allocation to eight: four drive + one flywheel + one intake + one transfer + one feeder. Pitch, compression and yaw use three Axons. This consumes the team's stated DC motor allocation; any additional powered subsystem requires a shared drive or a revised allocation.

## Current code already supports

- `ShotRecipe`: independent hood and compression commands, RPM tolerance, stable time, settling and calibrated range.
- `CalibratedShotTable`: ball-specific range interpolation and flight-time uncertainty, with no out-of-range extrapolation.
- `MotionShotSolver`: release delay, chassis translation/rotation, turret rate, muzzle offsets and a calculated miss bound. This is an empirical horizontal lead model, not a complete drag/spin/hive simulation.
- `ScoringController`: confirmed inventory, staged/exit events, RPM recovery, bounded recovery, held setpoints during a release and in-flight tracking.
- `BurstPolicy`: bounded overlapping flights through a supplied impact model.
- `BioBuzzTeleOp`: does not impose fixed scoring-pose restrictions when a shot table is used; recent changes already address that distinction.
- Exposure-time robot/turret histories, target freshness checks and fault handling.

These features should be retained. Their presence is not measured accuracy or competition readiness.

## Remaining changes, in priority order

### 1. Match yaw I/O to the Axon

`turret/RevTurretIO.java:10,37` still declares and maps a `DcMotor`. Introduce an Axon CR-servo implementation behind the existing turret I/O seam, retaining absolute acquisition, sign calibration, physical/operating limits, stopping bounds and final output checks. Standard positional servo commands and continuous-rotation power commands are different; do not substitute `setPosition(power)` for `setPower(power)`. CR mode and neutral/stop behavior must be configured and verified. Supply measured encoder transfer and zero/ratio data. `TurretHardwareConfig` currently has an unselected backend; PWM acquisition is explicitly unimplemented, while analog has a validation path.

### 2. Add launcher geometry and calibrated actuator units

`scoring/MotionShotSolver.java:13,59` uses one fixed horizontal muzzle offset. Introduce geometry indexed by pitch and compression/ball setting: release position relative to yaw pivot, release height, effective launch direction and actuator mapping. Keep servo normalization in the hardware/calibration layer; reason about pitch in degrees/radians and gap in millimeters. A fixed offset may remain an explicitly bounded approximation if tests demonstrate that its error fits the miss budget. Fixed-height empirical shot tables can remain useful without a full 3D physics solver.

### 3. Verify pitch/compression readiness physically

`robot/RevRobotIO.java:122` sends servo commands without measuring pitch or compression position. `scoring/ScoringController.java:210` relies on `settleSec` for actuator readiness. Add measured pitch, pitch velocity and validity to the hardware snapshot, preferably from output-side sensing. Gate feeding on achieved pitch error/rate. Give compression its own known-state/settling check, using sensing where available or explicitly characterized timing. Do not treat a commanded servo position as measured angle. Separate yaw, pitch, compression and RPM readiness in telemetry.

### 4. Keep two compression settings, populate real range tables

For the initial mechanism, hold each ball type's compression constant across its range knots. `CalibratedShotTable.java:47` interpolates compression today; supplying identical per-type values avoids unnecessary motion without rewriting the table. Only vary gap with distance after experiments show a benefit. Measure RPM, pitch, flight time, release delay and spread at multiple distances, plus intermediate validation points. Recalibrate after changing shoe surface, roller friction, wheel tread or release geometry. Two compression settings do not mean two total shots for the whole field.

### 5. Target a point inside the cell

`vision/pieces/TargetEstimate.java:6` stores x/y and a discrete side state. `CameraTargetTransform` projects the observed tag location horizontally; it does not identify a desired internal impact point or preserve target height. Add explicit cell geometry and a chosen impact point relative to its observed pose. Retain height/pose information as required, apply tag-to-cell offsets and produce a shot envelope that avoids the lip. Camera offset and launch-point offset are separate calibrations. Do not automatically aim at the AprilTag center as the optimal tipping point.

### 6. Separate reliable scoring from researched tipping bursts

Keep the current conservative shot mode. `ScoringController.java:165,199` still requires a settled hive even when an impact model is supplied. That model can authorize overlapping flights while the observed hive is stable, but it does not authorize targeting a moving hive. A researched fast-tip mode needs measured arrival-time target motion, shot spacing and impact behavior, with an explicitly valid arrival interval. Extend hive state beyond side labels only if required by those measurements. `planning/pickup/TipModel.java:14` accepts only carried ball counts; extend it to use the actual shot recipe, target/load state, shot spacing and uncertainty if tip optimization is the objective. Do not hardcode 3 nectar/4 pollen as guaranteed successes.

### 7. Remove avoidable cycle delays after proving feed isolation

Current non-burst confirmed shots and IDLE request zero flywheel RPM; consider a deliberate pre-spin/ready state with one actuator owner. Keep appropriate limits and measured readiness while avoiding a full spin-up for every single shot. `RobotCoordinator.java:220` inhibits intake outside scoring IDLE. If the physical ball path isolates collection/storage from the release stage, allow concurrent intake and shooting with conveyor ownership and queue updates that prevent a second ball being forced into the compression region. If the mechanisms are coupled, retain the interlock. Use actual RPM recovery to determine cadence rather than a universally reduced delay.

### 8. Add shot records and populate measured configuration

Record shot ID, ball type, configuration revision, robot velocity/acceleration, target pose/point, commanded/measured yaw and pitch, compression setting, RPM/droop/recovery, feeder start, exit timestamp, predicted flight interval/miss bound and observed scoring/tip outcome. Existing loop/state telemetry is useful but not a complete shot dataset. Add a safe bench calibration OpMode and a versioned measured profile. `RobotHardwareConfig.java:18` still returns null, so competition initialization intentionally remains disabled. Populate it after hardware measurements; do not use simulation fixtures as calibration data.

## Acceptance sequence

1. Assemble and measure guides, wheel stack, transmission, encoder coupling and complete size envelope. Verify smooth full motion and ball capture at intended extremes.
2. Measure static and shooting-load deflection, pitch backlash, compression repeatability, servo current/temperature and holding behavior. Keep the selected contact setting after repeated ball loads.
3. Characterize each ball type independently at stationary near/mid/far positions with fresh and worn balls; validate intermediate ranges and off-axis yaw positions. Record spread and misses as well as successes.
4. Test mixed-ball transitions, fast same-type recovery and jam recovery without sacrificing known inventory or exit confirmation.
5. Use the complete hive to compare ordinary scoring against candidate 3-nectar/4-pollen bursts across impact point, trajectory and cadence. Measure tip rate and rebounds; preserve a reliable ordinary scoring fallback.
6. Expand to constant-speed moving shots, then acceleration/rotation and controlled disturbance tests. Accept only the measured envelope that stays within the miss bound. Recheck battery and warmed-mechanism effects.
7. Rehearse full autonomous and driver cycles, including recovery and service access. CAD appearance and desktop scenario counts do not establish a Worlds-level shooter.

## Sources

- [Axon programmer and CR firmware](https://www.gobilda.com/axon-servo-programmer-mk2/).
- [AndyMark encoder outputs and mechanical interfaces](https://andymark.com/products/hex-bore-encoder).
- [Team 254 2022 technical binder, shooter p.17](https://media.team254.com/2022/10/8f2407a5-Team_254_Tech_Binder_2022.pdf): powered main and back rollers share shooter motors; a top roller does not inherently need another motor.
- Current repository source files listed above; Mark 3/Mark 4 geometry reports and user-selected hardware from this conversation. The final shortened carrier/carriage has not yet been modeled or load-tested.

## Verification during this review

`tools/run-desktop-tests.ps1` completed successfully: 1,878 checks across ten suites (planner 99, tracker 180, state history 23, turret safety 72, Pedro mapping 45, safety regression 498, turret bench 114, audit regression 38, robot integration 691, moving shot 118). These desktop suites exclude SDK-dependent adapters; no Android build or physical test was performed for this read-only review. Existing Java 8 target deprecation warnings remain. `git diff --check` passed. The companion design/code canvas passed TypeScript 5.9.3 validation against the installed SDK.
