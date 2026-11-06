# BIOBUZZ turret CAD and code review — 2026-10-05

## Assessment

The measured feed bore and replacement hood width clear the nominal sizes of both pieces. The current compression geometry does not establish reliable shooting of both. The replacement hood must be integrated with the actual wheel, its slider travel revised, and both settings characterized with real balls before finalizing the mechanism.

This was an inspection of the open Fusion designs `Ragnar Turret Mark 2` and `Hood Desgin Mark 2`, plus the current TeamCode source, including local uncommitted changes. Temporary component hiding and isolation were restored. No geometry or control source was edited during this review. This is not a completed interference study, launch-performance test, or robot inspection for rules compliance.

## Dimensions and implications

AndyMark lists nominal pollen diameter as 2.8 in (71.12 mm), and nectar as 3.6 in (91.44 mm). Their nominal diameters differ by 20.32 mm. These conversions do not imply that actual balls have hundredth-millimeter precision. [Scoring elements](https://andymark.com/products/biobuzz-scoring-elements?variant=47194281181356). The pollen preview specification also lists ±0.1 in diameter variation; measure the actual game balls rather than tuning only to the nominal number. [Pollen preview specification](https://andymark.com/products/ftc-2026-27-game-preview-pack).

| Fusion measurement | Value | Interpretation |
|---|---:|---|
| Stationary transfer tube, inner circular edge and cylindrical face | 100.00 mm diameter | Nominal nectar has 8.56 mm diametral clearance, or 4.28 mm per side when centered. This does not check the feeder gate or every transition. |
| Replacement hood, opposing inner planar faces of the gray side carriers | 106.00 mm separation | Nominal nectar has 14.56 mm lateral clearance. Guides, screws, and brackets must preserve this passage. |
| `04_FLYWHEEL:1`, isolated cylindrical contact surface | 50.00 mm radius / 100.00 mm diameter | Confirmed on the tread surface, separately from nearby gears. Diameter alone cannot establish adequate launch speed or recovery time. |
| Old `P1_shoe_carriage_and_rack:1`, inner circular arc | 119.00 mm radius | With the wheel and shoe concentric in the shooting plane, the radial gap is 69.00 mm. |
| Replacement `HOOD_compression:1`, inner circular arc | 135.44 mm radius | A concentric installation with the 50 mm wheel gives an 85.44 mm gap at this insert position. This is conditional on final mounting alignment. |
| Replacement insert, exposed outer cylindrical face | 149.44 mm radius | This outer face is not the ball-contact radius. |

For nominal balls, define geometric squeeze as `ball diameter − contact gap`. Positive means interference/compression; negative means clearance. This does not predict contact force or launch accuracy.

| Contact gap | Pollen | Nectar |
|---|---:|---:|
| Old shoe: 69.00 mm | 2.12 mm squeeze | 22.44 mm squeeze |
| New insert, aligned concentric installation: 85.44 mm | 14.32 mm clearance | 6.00 mm squeeze |

The old setting is a plausible pollen test point, not a validated optimum. I would not approve its 22.44 mm nominal nectar squeeze without substantial physical evidence. The new setting is a plausible nectar test point, but it leaves pollen without opposing contact at that surface.

The new gap must decrease by 16.44 mm to reproduce the old 69 mm pollen gap at the critical contact location. To give both nominal sizes the same absolute squeeze, their effective gap settings must differ by 20.32 mm. Different optimal squeeze for the two balls changes that difference.

The existing `Compression_ForeAft_Trial_10mm` joint is a motion concept, not validated travel. Even a perfectly aligned 10 mm inward translation would leave a 75.44 mm local gap from the new 85.44 mm starting point, above nominal pollen diameter. A translation of a curved insert also moves its center away from the wheel center: gap change varies along the arc, and generally is not equal to slider travel everywhere. Determine stroke from assembled geometry at the entry, compression zone, and exit; include tuning allowance and actual ball variation.

Fusion's minimum distance between circular edges in different axial planes is a three-dimensional edge distance. It must not be substituted for the radial ball-contact gap.

## Mechanical changes to prioritize

1. **Integrate the new hood with the actual wheel first.** Use common reference geometry for the wheel axis, hood pivot, and ball centerline. Check both ball envelopes through the feed transition, both compression settings, and the intended hood-angle range. Check the cage, slider hardware, link, and wiring as well as the yellow shell.
2. **Make compression independent of aiming.** The guides and servo should be mounted to the carrier that follows both turret yaw and hood pitch, with the insert moving relative to that carrier. A servo on a frame that follows yaw alone can couple compression to hood pitch unless the linkage explicitly compensates for it.
3. **Design both settings to withstand the actual ball-force direction.** An ordinary pair of opposite travel end stops does not automatically support two firing positions: if ball pressure pushes outward, the inward travel stop cannot resist it. Use two indexed load-bearing positions, a properly arranged latch/cam/linkage, or another positively supported arrangement. Verify the load path and force direction in both positions. Choose the Axon model and horn geometry after stroke, force, and available space are known.
4. **Support the insert across both sides.** Use a stiff spanning carriage, short links, replaceable contact material, and serviceable fasteners. Keep mounting hardware outside the ball passage. Avoid forcing imperfectly parallel guides to fight each other; a fixed/floating guide arrangement is one option. [igus guide design notes](https://www.igus.com/linear-bearings/linear-guides-drylin-w-design-notes-com).
5. **Build a measured single-piece feed path.** Confirm a staged piece, distinguish its type, meter one piece, and confirm exit. Add a bounded jam-clear operation and accessible service points. A floor-camera observation does not prove what is currently at the shooter.

## Where the current code matches and where it stops

The repository has tested pickup planning, piece tracking, pose handling, turret aiming, rear-sector recovery, and bench hardware interlocks. It does not yet contain an integrated competition shooter/hood/compression/feeder subsystem or competition autonomous/TeleOp executor in TeamCode.

| Source evidence | Consequence for this CAD |
|---|---|
| `turret/TurretHardwareConfig.java`: sensor backend `UNSELECTED`, zero and analog transfer values `NaN`, confirmation flags false, closed loop disabled | Select and characterize the actual sensor and output-side mounting before enabling motion. |
| `turret/TurretConstants.java`: placeholder physical ±165°, operating ±160° | Replace with measured cable and collision limits, including stopping margin, after the hood/servo/wiring are installed. |
| `turret/RevTurretIO.java`: `setMotorPower` calls `command` with arming and deadman false | This adapter currently supports the gated bench path. A competition caller using only the generic setter will not drive it. Integrate a deliberate competition output gate; do not bypass protection indiscriminately. |
| `turret/TurretController.java`: bearing from robot pose to target, with bearing-rate/chassis-yaw tracking | It aims in yaw. It does not calculate hood angle, ball compression, wheel RPM, release timing, or projectile motion lead. Include measured turret-origin and release-point offsets when building the shot solver. |
| `planning/pickup/PickupPlan.java`: `SHOOT_NOW` documented as strategic intent | The future executor still needs staged-piece confirmation and shooter readiness before feeding. |
| `planning/pickup/PreferredPoseShotSetup.java`: accepts but ignores `BallLoad`; returns nearest-pose travel estimate plus fixed shot delay | It treats different loads similarly and does not communicate a selected firing pose/recipe. Add shot selection and type/order-dependent execution cost. |
| `planning/pickup/PickupPlannerConstants.java`: `SHOT_EXECUTION_SEC = 0.80` is a placeholder | Measure compression switching, spin-up, per-shot recovery, and feeding. Otherwise the planner can overvalue a route that is slow to unload. |
| `pedro/PedroConstants.java`: robot-specific drivetrain and localization values explicitly unmeasured | Calibrate the installed drive and odometry before trusting field-relative turret aim or pickup routes. |
| `vision/pieces/PieceTrackerConstants.java` and `HomographyFloorProjection.java`: taught IDs and measured floor calibration needed | Give the pickup camera a rigid mount. Moving it with the hood would require a changing projection rather than a single fixed floor map. |

## Recommended shooting integration

Use a calibrated shot recipe keyed by piece type and target/range: hood angle, compression position, flywheel RPM, and minimum recovery/settling criteria. Separate selection of that recipe from executing it.

A practical first executor sequence is `stage → identify → configure → spin/aim → verify ready → feed one → confirm exit → recover`. Feeding should require a valid aim solution, low aim error and angular rate, settled hood/compression, wheel RPM within tolerance, and a confirmed staged piece. Sensor failures or timeouts should cancel feeding and report a specific fault.

Connect rear-sector chassis recovery to the actual drive coordinator. The turret's chassis-omega output is a request; it does not move the drivetrain by itself. Keep feeding inhibited while the coordinated recovery changes the firing solution.

Start with reliable stationary shots. Tracking a changing bearing is different from compensating for robot velocity during projectile flight. Add moving shots only after measuring release delay and flight behavior, and model velocity at the actual release point, including rotation-induced motion.

For planning, use measured scoring success and cycle time for the actual piece mix, not just nominal travel time. Include time to change compression between types and recover wheel speed; batch equal types only if the real storage/feeder can select or reorder them. Calibrate `MapTipModel` from observed outcomes for the relevant game state.

## Validation performed and next measurements

The existing `tools/run-desktop-tests.ps1` completed cleanly: seven scenario suites, 1,031 checks total, plus the desktop reproduction scenarios. This verifies the covered software behavior; it does not validate motor performance, ball compression, shot accuracy, or final mechanical integration.

Next, build a small test matrix with actual pollen and both nectar colors: measure ball size, contact gap, compression force, wheel RPM before/after a shot, exit speed or landing location, recovery time, and jams. Sweep gap and RPM at fixed hood angles, then build range tables from repeatable settings. Record multiple-ball sequences and partially discharged battery behavior. Acceptance targets should be set by the team's required scoring rate and reliability.

Remaining unverified items include the final assembled new hood, slider and servo hardware, minimum clearance through every feed transition, full yaw/pitch interference sweep, actual actuator capability, complete robot dimensions, and current game-rule compliance. No blanket competition-ready signoff follows from the inspected dimensions or desktop tests.
