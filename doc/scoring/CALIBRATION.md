# Range, motion and burst shooting

The competition configuration lives in `RobotHardwareConfig.Settings`.
`RobotHardwareConfig.measured()` still returns `null`: supply actual measurements
and bench acceptance before running mechanisms. Desktop scenario values are
synthetic and are not robot settings.

## Range calibration

Set `shotTable` to a `CalibratedShotTable` with at least two distinct
`CalibratedShotTable.Point` entries for each `BallType`. Each point contains:

- Horizontal range in inches, a measured `ShotRecipe`, muzzle-to-impact flight
  time in seconds, and a bound on flight-time variation.
- RPM, normalized hood/compression commands, servo settling time, RPM tolerance
  and continuous RPM stability time in the recipe.

RPM, hood, compression and flight time interpolate between adjacent knots.
Settling uses the larger neighboring time and RPM acceptance uses the tighter
neighboring tolerance. Outside the measured endpoints, shooting is unavailable.
Only put neighboring knots in one table when the intervening combinations have
been characterized. A different target height or launcher geometry needs its own
characterization. Duplicate fixed recipes are rejected rather than silently
overwriting a ball type.

With the moving solver, range is measured from the muzzle to the target. Without
it, stationary selection uses the turret pivot as its range reference. Calibrate
against the reference actually configured; do not reuse a pivot-range table as a
muzzle-range table without correcting and validating it.

Set `rpmRetargetFraction`, `hoodRetargetTolerance` and
`compressionRetargetTolerance` from the measured acceptance envelope. Small
changes hold the previous actuator commands; larger changes update them and
restart servo/RPM settling. This permits continuous range changes when the held
command remains within the characterized envelope. Commands stay fixed during
each release; a solution outside that envelope stops the release and requires
inventory reconciliation. Flight-time variation and the moving solver's
`calibrationErrorInches` must include this envelope, interpolation error, accepted
RPM variation and physical shot dispersion.

## Release timing and moving shots

Configure `movingShots` with `MotionShotSolver.Settings`. Measure command-to-muzzle
release delay and its uncertainty, muzzle forward/left offset relative to the
turret pivot in the turret frame, target-position uncertainty, calibration error,
and the allowed miss bound. The camera transform supplies the measured chassis
to turret-pivot offset. All position/velocity units are inches and seconds; angles
are radians, positive counterclockwise, in the existing project field frame.

The solver predicts chassis position, heading and velocity at release using
acceleration estimated from successive localizer samples. It rotates the pivot
with the chassis and the muzzle with both chassis and turret motion. Release
velocity includes both rotation-induced contributions. It solves:

`equivalentRange = |target - muzzleAtRelease - muzzleVelocityAtRelease * flightTime(equivalentRange)|`

The table at that equivalent range supplies RPM and hood. The corresponding
compensated bearing drives the turret. The coordinator predicts the physical
barrel bearing at release, keeps the predicted encoder angle inside the turret
operating interval, and commits the release timestamp when feeding begins.
Nonconvergent or out-of-table solutions cannot authorize feeding.

`MotionConfidence.Limits` requires measured freshness, speed, yaw-rate,
acceleration, angular-acceleration, pose/rate residual, recovery-duration and
position/velocity/heading/yaw-rate uncertainty bounds. The solver also requires
turret rate, acceleration and rate-uncertainty bounds. Freshness alone does not
establish confidence. Distinct consistent acquisitions must persist through the
recovery interval; pushes, pose/rate disagreement, excessive rates, reset
generations and stale samples inhibit firing immediately. Estimated miss includes
calibration/target error, motion uncertainty, possible acceleration during release
delay, launcher rotation, timing uncertainty and the measured predicted alignment
error. Their sum must fit `maxMissInches`.

One localizer cannot detect slip that makes its pose and velocity agree with the
same incorrect motion. Characterize uncertainty against an independent field
reference, including collisions and pod/wheel slip. If those bounds cannot fit
the target opening, moving fire must remain inhibited. These checks are a bounded
prediction gate, not a calibrated probability of scoring.

## Sensed exits and bursts

Set `exitLatencySec` to a measured upper bound from physical muzzle exit to the
debounced event observed by the controller, including sensor placement,
debounce, SDK read and loop latency. Set `impactSettleSec` to the measured
post-impact HIVE motion/settling bound. Both apply to calibrated single shots
as well as bursts. Confirmed flight arrival intervals start at
`observedExit - exitLatencySec` through `observedExit`, then include measured
flight time and its variation. A confirmed exit inconsistent with the committed
release interval latches a timing-calibration fault and retains the flight.

Configure `burst` with `BurstPolicy` to continue while A is held. Its release
delay/uncertainty must match the moving solver when both are configured. Use
`maxInFlight = 1` and a null impact model for conservative operation. The wheel
stays spinning after an exit, the next FIFO piece gets its own recipe, and the
controller requires stable recovered RPM, settled actuators, a clear exit sensor,
fresh aim/HIVE observations and a safe flight window. It does not reorder balls.

With the conservative policy, a next release waits through the previous ball's
latest possible arrival plus impact settling, and then requires observations
acquired after that interval. Burst HIVE timeout starts after expected arrival
and settling, so valid flight time does not consume the reacquisition timeout.
Release A to stop preparing the next burst shot; a current release still awaits
exit confirmation. B cancels immediately. A new press is needed to rearm. The
autonomous single-shot pulse remains a single shot even with burst settings
installed: burst continuation is an explicit input request from TeleOp. Confirmed flights
survive cancellation and localization resets until their intervals expire.

Overlapping flights require a supplied, measured `BurstPolicy.ImpactModel` and a
bounded `maxInFlight` greater than one. The model receives the next piece type,
target ID, current HIVE state and observation time, the entire candidate arrival
interval, and all pending flight intervals/types/targets. It must certify the
candidate window accounting for intervening impacts and tips. No optimistic
default model is supplied. UNKNOWN/TIPPING/stale observations still inhibit
feeding. Partial-release cancellation/faults retain a possible flight; even an
impact model cannot bypass that uncertain flight by reconciling inventory.

A wheel-speed drop during feeding stops the feeder and waits for the sensed exit.
It neither decrements inventory nor automatically feeds again. Missing exits time
out and require reconciliation. Bounded reverse recovery is confined to preparation;
a partly released shot cannot be retried based only on a staging sensor.

## Measurement and verification

Record repeated stationary shots at the endpoints and intermediate ranges for
both ball types. Include accepted ball variation, battery state, RPM before/after
release, commanded/actual timing, landing dispersion and HIVE response. Verify
the retarget envelope by deliberately shifting range while commands are held.
Then measure forward, reverse, lateral and turning shots against an independent
motion reference, including pushes and slip. Verify inhibition and recovery as
well as scoring accuracy. Finally measure mixed-type bursts and worst-case tip
sequences before supplying an overlap model.

TeleOp shows the equivalent range, estimated miss bound, confidence/inhibit reason
and balls in flight. Use `tools/run-desktop-tests.ps1` for deterministic regression
checks and `gradlew.bat :TeamCode:compileDebugJavaWithJavac` for Android compilation.
These checks validate software behavior; physical acceptance comes from the
measurement runs above.
