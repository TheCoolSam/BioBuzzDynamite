# Launcher calibration

The Java implementation now supports Axon CR yaw, pitch/gap-dependent release geometry, output-side launcher feedback, separate physical-unit shot tables, internal cell points, pre-spin and isolated concurrent intake. The CSV files are empty measurement templates. They cannot arm the robot. `RobotHardwareConfig.measured()` still returns `null` until physical measurements are supplied.

Independent `BIOBUZZ Drive Practice` and `BIOBUZZ Stationary Scoring` modes,
persistent autonomous shot actions, measured wall departure, deliberate drive-only
fault recovery and observer outcomes are documented in
[`COMPETITION_READINESS_IMPLEMENTATION_2026-10-11.md`](../../doc/audits/COMPETITION_READINESS_IMPLEMENTATION_2026-10-11.md).
Stationary scoring has its own measured profile and does not require moving-shot
tuning or floor pickup planning. The profile factories remain null pending measurements.

## Yaw

Configure the Axon with CR firmware and configure `turretYaw` as a CR servo in the robot hardware map. SENSOR_ONLY can run with no CR calibration: the analog sensor is readable and output is impossible. Use the bench CR_DEADBAND step to measure neutral and the low/high pulse deadband, then paste those values into `TurretHardwareConfig.CR_CALIBRATION` with the programmer low/high/frame. Verify stationary neutral at operating voltage before setting `CR_MODE_AND_NEUTRAL_VERIFIED`. SDK zero power is not assumed to be the measured neutral. Sign-test power is derived from that deadband and is refused if 0.03 would sit inside it.

Stop and E-stop write the calibrated neutral pulse and then disable PWM on that port. A CR servo has no brake: PWM disable can backdrive, so mechanical stops and operator power removal remain required. Uncalibrated CR never writes SDK zero. Recharacterize stopping distance and the control profile for this actuator.

Retain the existing encoder transfer, zero/sign, output coupling, limit, deadman, age and power checks. Defaults prohibit motion. The motor allocation is four drive, flywheel, intake, transfer and feeder: eight DC motors. Yaw, pitch and compression use servo ports.

References: [Axon CR firmware and programmer](https://www.gobilda.com/axon-servo-programmer-mk2/), [FTC CR servo PWM control](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/11.0.0/com/qualcomm/robotcore/hardware/CRServoImplEx.html).

## Geometry and sensing

Measure release forward/left offset and height (inches from the yaw pivot, height above the floor), and effective launch pitch (radians) on a grid of output pitch (radians) and shoe gap (mm). Include both ball gaps, all intended pitch travel, loaded deflection and interpolation checks. Construct `LauncherGeometry` with this grid, a conservative position error, and monotone physical-unit-to-servo `CalibratedAxis` curves. Its coordinates use forward/left/up. Curve inputs and outputs must cover the entire grid. Servo values stay in [0,1].

Map independent analog **output-side** sensors using `pitchSensor`, `gapSensor`, `pitchFeedback` and `gapFeedback` in `RobotHardwareConfig.Settings`. The feedback curves convert measured volts to output pitch radians and gap mm. Verify their transfer and loaded mechanical coupling, then set `outputSideLauncherSensingVerified`. The first sample and invalid/stale/discontinuous samples do not establish zero velocity. Configure separate pitch error/rate and compression gap error/rate limits plus measured stable time. Sensor faults inhibit feeding; loss of readiness during release faults inventory. The final SDK writer rechecks physical positions/rates before forward feeding. Reverse recovery remains bounded by the existing mechanism interlocks.

Readiness telemetry reports pitch, compression and RPM independently. `Servo.getPosition()` is never used as physical feedback. The retained normalized recipe constructors serve existing desktop fixtures; competition configuration requires physical recipes.

## Shooting measurements

Fill `pollen-shots.csv` and `nectar-shots.csv` independently, with at least near/mid/far distances and more knots wherever measured behavior needs them. Keep `gapMm` fixed throughout each type's initial table. Pitch and RPM may vary with distance. Record repeated trials and validate intermediate pitch/RPM combinations before accepting interpolation. Two compression settings are not two shooting settings.

Columns are range inches, RPM, output pitch degrees, shoe gap mm, actuator settling seconds, accepted RPM error, RPM stable seconds, release-to-impact flight seconds, flight uncertainty seconds, feeder-start-to-release delay seconds, release uncertainty seconds, impact height inches and accepted height deviation inches. Range is measured horizontally from the actual release point, including pitch/gap-dependent offsets. Flight and release bounds must cover worn balls, load, battery and sensor/debounce latency. Release delay is per knot; solver settings supply the maximum allowed delay/uncertainty across the tables. Release delay plus uncertainty must fit the measured feed pulse.

Use `ShotCalibrationCsv.load(pollenReader, nectarReader, geometry)` to produce the table for the measured settings. It rejects empty/nonfinite data, missing types, duplicate ranges, varying initial gap and incompatible geometry/height within a type. It does not extrapolate. Reader/file acquisition is caller owned; there is no automatic download or default calibration. The table is calibrated at a fixed impact height per type; an uncharacterized height inhibits a shot. Release height is preserved in the solution, but this remains an empirical flight-time model, not a 3D aerodynamic solver.

## Cell points and tip estimates

Measure `CellTargetGeometry.Point` for each allowed tag and stable hive side, in the tag's local coordinates. Choose a point inside the cell; its `clearanceInches` is the **remaining center miss allowance after accounting for ball radius, the lip and desired margin**. Validate this envelope over the accepted trajectory and pose uncertainty. The coordinator limits its horizontal miss budget by this allowance. This is not an automatic lip collision model.

Supply the optical-to-turret rotation, camera offsets and camera height through the geometry-aware `CameraTargetTransform` constructor. Camera translation and muzzle translation are separate. Limelight `targetpose_cameraspace` / fiducial `t6t_cs` is `[tx, ty, tz, pitch, yaw, roll]` in camera space (X right, Y down, Z forward). FTC SDK 12 `Pose3D` helpers construct `YawPitchRollAngles(yaw=a[5], pitch=a[4], roll=a[3])`, so `getYaw/getPitch/getRoll` are Limelight roll/yaw/pitch. `LimelightCameraPose` rebuilds `Rz(yaw)·Ry(pitch)·Rx(roll)` from the documented array order. Desktop tests keep a CELL point fixed at turret yaw ±30° and reject the SDK-labeled mapping at pitch ±30°. This still needs a physical check with a real tag at ±30°. Missing tag/state geometry produces no shot point; an AprilTag center is not substituted.

`TipModel.getTipProbability(ShotTipContext)` takes the recipe, impact point, observed hive side, carried load, arrival interval, cadence and pending flights. Its default is unknown (`NaN`); count-based planner estimates cannot establish shot tipping performance. Override it with measured behavior. Physical overlapping-flight models must override `BurstPolicy.ImpactModel.safe(ShotTipContext)`; the default is false. Moving-hive shooting remains inhibited by the stable-hive gate until a separately measured motion model is implemented.

## Cycle optimization and logs

Driver controls: hold RB to enable; A starts an ordinary shot; G2 LB deliberately pre-spins without transfer/feeder outputs; G2 RB plus held A requests a separately validated burst. Releasing enable or cancelling stops outputs. Pre-spin retains measured readiness for an unchanged next recipe. Gaps/pitches still need independent readiness and RPM recovery.

Set `feedStagesIsolated` only after physically proving collection/storage cannot push an extra ball into the release stage while transfer/feeder run. Intake remains gated by confirmed capacity, entry classification and enable; it stays inhibited during reversal and faults. A same-cycle exit does not create speculative intake capacity. Transfer/feeder retain one owner.

Ordinary scoring remains available when burst validation is missing. Physical bursts require separate `pollenBurstValidated`/`nectarBurstValidated` flags, stop after four pollen or three nectar exits and stop at a type transition. These are trial size limits, not guaranteed tip counts. The full shot/impact model must authorize overlapping flights; observation freshness, measured RPM recovery and launcher readiness still apply.

`BioBuzzShot` RobotLog entries include calibration revision, shot ID, feeder-start/exit timestamps, robot motion/acceleration, cell point/height, command/measured yaw/pitch/gap, RPM/minimum/recovery, arrival prediction, miss bound and state/reason. State transitions and exit edges are always recorded, with samples at 20 Hz during operation. The journal retains at most 512 pending rows and reports dropped rows. TeleOp drains it to the robot log. `ShotJournal.outcome(id,time,score,tip)` appends an observed outcome; an exit sensor does not establish a scored ball or tip. Join external video/field observations by shot ID when evaluating calibration and burst trials.

Rows now also append `runId`; join by run ID, calibration revision and shot ID.
Observer controls label the displayed last confirmed release: G2 A hit / B miss,
left stick click tip / right stick click no tip. These are observations, not
automatic scoring claims. Labels can remain unknown and can be supplemented from video.

Recalibrate after changes to the shoe, wheel, roller friction, geometry, sensor coupling or actuator programming. Desktop scenarios and Android compilation verify software only. Physical shooting tables, stopping/load tests, sensor calibration, feed-isolation acceptance and rapid-burst measurements remain necessary before filling the measured configuration.
