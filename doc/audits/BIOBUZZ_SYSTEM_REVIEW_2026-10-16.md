# BioBuzz / Ragnar system review — October 16, 2026

**Assessment:** The architecture is plausible, but the current assembly is a development prototype with one confirmed transfer collision, extremely small sizing margins, and no demonstrated end-to-end scoring performance. Those are the immediate obstacles to an elite robot. CAD cannot establish a world top-ten ranking; that requires match data and comparable opponents. I would retain the core turret concept and prove the ball path before adding complexity or printing the complete robot.

The team reports that only the helical gear, rack, and connecting rail have been printed and worked. This is useful fit/function evidence, not yet a loaded shooter or durability test.

## Scope and evidence

- Inspected the live Fusion MCP supplied by the team. Open design documents: **Ragnar Intake + Ball Path**, **Ragnar V1 Turret DEVELOPMENT**, **Pollen**, and **Blue Nectar**. Searched the project to distinguish current designs from archived Mark 4 alternatives.
- Intake: 1,312 timeline items, no reported timeline warnings, 277 visible occurrences, 569 visible occurrence bodies. Turret source: 669 timeline items, 79 visible occurrences, 199 visible occurrence bodies; one nonhealthy timeline group named `Group1` returned an empty message. That group is a CAD maintenance item, not evidence of a physical failure.
- Used world-context body proxies and temporary BRep copies for the new clearance calculations. All four open documents remained unmodified. Did not change geometry, joints, saves, visibility, or robot code.
- Evidence: [live inventory](C:/Users/sbots/StudioProjects/ftcrobotcontroller/tools/fusion/season_review_2026_10_16/inventory.json), [new clearance calculations](C:/Users/sbots/StudioProjects/ftcrobotcontroller/tools/fusion/season_review_2026_10_16/clearances.json), [motion limits](C:/Users/sbots/StudioProjects/ftcrobotcontroller/tools/fusion/season_review_2026_10_16/motion_metadata.response.json), [current view](C:/Users/sbots/StudioProjects/ftcrobotcontroller/tools/fusion/season_review_2026_10_16/live-intake.png).
- The intake contains a copied turret snapshot. Its motion is not live-linked to the turret source. The intake has no native joints in the inspected assembly. Existing subsystem sweeps do not constitute a combined robot motion test.

“Confirmed” below means directly measured or inspected. “Risk” means an engineering concern that needs hardware or further CAD validation; it does not mean a proven failure.

## 1. Correct the Nectar/turntable intersection first

**Priority: build blocker. Confirmed geometry conflict.**

Rechecked the saved four-Nectar queue positions against the current source turntable bodies. The second ball, diameter 91.44 mm, is centered at `(0, -78.5014, 80.72)` mm. It intersects four turntable bodies. Individual overlap volumes are approximately 18,519, 9,360, 1,778, and 8,264 mm³. These are individual body results, not a union volume to sum. This is substantial interference, not print tolerance. It means the specified four-Nectar storage layout cannot exist as drawn with rigid nominal balls.

**Preferred next experiment:** raise the turntable relative to the fixed intake path by 30–35 mm in a candidate assembly. Both heights cleared these four parked spheres in the new calculation. That is a promising candidate, not a completed fix: moving balls, the yaw handoff, feed roller engagement, hood motion, chassis height, and wiring still need checking.

**Alternatives:** lower or reshape the queue/rise while retaining floor clearance; move the bend forward so balls enter the turntable bore vertically; redesign the turntable support to leave an unobstructed approach. Lowering storage trades against ground clearance; raising the turret trades against height and center of gravity. Select these together with the chassis. Do not simply deform Nectar around the plate or assume tread pressure will overcome it.

**Acceptance:** every nominal Pollen/Nectar sequence clears all noncontact structure, followed by powered transport tests with real balls and realistic queue pressure. Deliberate tire compression is evaluated separately from hard-structure interference.

## 2. The robot has almost no remaining packaging margin

**Priority: build blocker before freezing chassis. Confirmed dimensions; full legality unverified.**

At the modeled 115° pickup/lip fold, the visible solids, including the static turret snapshot, occupy a bounding envelope of approximately **452.00 × 456.26 × 387.99 mm**. The deployed envelope is approximately **452.00 × 587.97 × 390.99 mm**. These are spans of the inspected solids, not floor-referenced full-robot heights or a certification of sizing. The folding calculation rotates temporary copies about the existing modeled pivots; it does not prove the folding path is free of collisions.

An exact 18-inch dimension is 457.2 mm. The folded layout therefore leaves **5.2 mm total width allowance** and **0.94 mm total fore/aft allowance**. At 90° fold its length is still approximately 501.96 mm, so a generic “folds upright” assumption is insufficient. The current live width is 452 mm; the older 448 mm note is stale.

**Recommendation:** insert the actual drivebase, wheel modules, battery, hubs, motor mounts, connectors, cable bends, and fastener heads into one master assembly now. Provide positive deployment/stow stops. Aim to recover meaningful manufacturing/service margin rather than spending the last fraction of a millimeter. A 10–15 mm total allowance is a reasonable design target to investigate, not a FIRST requirement. Move the fixed landing pan/rise rearward or shorten the forward layout before narrowing the effective pickup width indiscriminately.

The modeled deployed length can fit a 24-inch operating dimension in isolation. That does not validate all robot poses or its physical expansion constraints. FIRST TU05 R102 specifies the starting cube, and R105 specifies the operating envelope and physical constraints. [Official manual](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU05.htm).

## 3. Prove driven contact through the curved rise

**Priority: high. Unresolved transport risk.**

The conveyor is a reference 104-link, 8 mm pitch system with pressure rollers and a curved return/rise. Existing notes explicitly leave actual tread identity, reverse bending, engagement, contact pressure, and continuous driven contact unresolved. They record that the team has Badlands and an unidentified smaller softer tread. A drawn track is not proof that the purchased tread can follow this path.

Likely failure modes are loss of grip at the rising transition, Nectar wedging under a pressure arm, Pollen escaping the contact region, and a following ball driving the first into an obstruction. They must be tested rather than assumed.

**Preferred alternative:** a short transparent-sided test module using the exact tread and sprockets you own, with adjustable pressure-roller preload and a replaceable low-friction guide. If back-bending is poor, use a straighter compliant conveyor and a separate powered lift/transfer, or successive powered rollers with overlapping contact zones. That adds a transfer but may eliminate a fragile curved-chain assumption.

**Acceptance:** all 16 four-ball size sequences, worn and new balls, starts/stops, reversing, and two balls arriving side by side. Record jams, damage, and transit times under power. Start with 100 cycles per challenging sequence as a screening test; zero failures there is not proof of match-level reliability.

## 4. The final spring-opening feeder needs a loaded force test

**Priority: high. Unvalidated mechanism.**

The two final 48 mm wheels use moving carriers, spur pairs, miter gears, and spring closing. The drive reaction can open a carrier. The CAD includes bearings, keepers, thrust reaction provisions, springs, stops, and tethers; I am not treating those as missing. Their dimensions and spring settings still need physical validation.

Too little preload loses grip when the motor applies torque; too much increases deformation, current, friction, and exit variability. A wheel that touches a ball in CAD may unload while driving.

**Recommendation:** bench one complete driven side with its actual spring, bearings, thrust spacers, gears, and ball contact. Measure the force and deflection while feeding both ball sizes. Use adjustable anchors and positive opening limits. Keep the two sides mechanically aligned.

**Alternative:** a simpler belt-driven pair of compliant rollers, with one spring-loaded roller and a tensioner arranged to accommodate its travel. Compare force consistency, reverse behavior, package size, and service time before switching. Purchased miter gears are not inherently a problem; replacing them with printed bevel gears would add uncertainty.

## 5. Add deliberate staging and sensing before chasing cadence

**Priority: high. Integration risk.**

Four-ball spacing is currently a geometric layout, not a demonstrated single-ball escapement. Sensor interfaces are reserved, but the actual sensor bodies, brackets, beams, wiring, and time response are not established in the inspected model. The next ball must stay clear of the flywheel when the feeder stops, including coast-down and pressure from the queue.

**Recommendation:** define a real staging pocket, independent feed release, and entry/stage/exit observations. Prove one command releases one ball and that the second ball stays out of the contact arc. The existing separate transfer and final-feed motors are useful for this; retain isolation until testing shows they can be combined.

**Alternative:** a small mechanically gated escapement if stopping the nip wheels cannot retain the queue consistently. It adds actuation and parts, so use it only if the simpler staged feeder fails.

## 6. Mixed-ball compression is a real speed tradeoff

**Priority: performance limit, conditional on the intended shot interval. Calculated, not measured.**

The current Mark 4 contact settings use a nominal 20.32 mm size change and a 24T MOD0.8 compression pinion. Pitch radius is 9.6 mm, so this change needs approximately **121.3° of servo rotation**. The live Mark 4 slider limits are −2 to +24 mm; an old hidden V1 22 mm joint still exists. Do not confuse its historical stroke with the nominal Mark 4 size change.

Using Axon's published MAX MK2 speeds, ideal unloaded size-change time is approximately **0.172 s at 8.4 V**, **0.232 s at 6 V**, or **0.283 s at 4.8 V**. Loaded movement and settling add time. Therefore a full compression change cannot itself finish within 0.15 s at those published speeds. Same-size bursts do not need that change. Actual ball release spacing also depends on when the previous ball leaves the contact region and what operations can overlap.

**Preferred alternative:** establish fast same-size bursts first and allow slower mixed transitions. If mixed speed materially changes match performance, investigate a larger pinion or shorter required travel, retaining sufficient force and stiffness. A larger pinion reduces available rack force at the same servo torque and needs a new mesh/mount check. Another option is a two-position selector or a passive compliant contact surface, but either must match the accuracy of the present adjustable hood before replacing it.

The full modeled 70° pitch range corresponds to about 303.3° input motion: approximately 0.43 s unloaded at 8.4 V or 0.58 s at 6 V. Small aim corrections are faster. This is a reason to minimize unnecessary aim changes, not to discard pitch control. [Axon specifications](https://docs.axon-robotics.com/servos/max).

## 7. Prove the shooter on a rigid bench with the real HIVE

**Priority: highest performance uncertainty. No firing evidence yet.**

The two 72 mm Hogbacks and steel inertia disc are a credible starting point. The printed gear/rack/rail results support continuing with them. They do not establish loaded backlash, pitch holding accuracy, compression repeatability, flywheel recovery, or HIVE tipping.

**Recommendation:** build the minimum complete fixed bench launcher before the complete intake: real shaft/bearings/hubs, motor and final transmission, hood, compression actuator, and output position measurements. Test a fixed position first, then pitch. Use the real HIVE with its pivots/dampers. Record ball type, ball identity/condition, range, launch setting, measured RPM before/after release, release interval, hit point, and tip result.

Measure scatter and tip success over repeated attempts, including successive volleys. Four Pollen or three Nectar causing a tip remains an experimental target. Impact location, retained balls, HIVE movement, and ball deformation matter. A smooth CAD trajectory or high wheel RPM cannot establish the result.

**Alternative:** retain adjustable pitch/compression but compete initially from a few calibrated stationary shooting locations. Add range interpolation and moving shots only after the fixed-location results support them. This can produce useful match performance sooner while preserving the turret architecture.

## 8. Finish the actual power and sensing chain

**Priority: build/integration blocker. Confirmed missing competition configuration; some CAD hardware incomplete.**

The inspected intake has three modeled DC drives. With four drive motors and one flywheel motor, that consumes eight motors. The live mouth sweepers share pickup drive through miter gears; the older description of two dedicated sweeper servos is stale. Deployment/counterbalance still needs a practical solution; any added motor must fit the actual resource budget.

The visible turret source includes an encoder body and mount, but its final driven transmission and yaw actuator are not established by the visible current assembly. The main flywheel motor/transmission is also not finalized in that source. These must be placed with real shaft retention, connectors, and access, rather than treated as negligible additions.

Current code already includes **CRServo yaw, calibrated shot tables, motion compensation structures, and burst logic**. Older claims that those features are entirely absent are stale. However, `RobotHardwareConfig.measured()` and `stationaryMeasured()` still return null; the yaw backend remains unselected and its calibration/enable flags remain unconfirmed. That is appropriate for untested hardware. It means this checkout is not a configured competition robot.

**Recommendation:** freeze a wiring/actuator list and implement calibration around the physical build. Include output-side pitch/compression verification, yaw feedback, sensor placements, motor SKU/RPM, ratios, current, and servo supply voltage. Do not enable competition behavior using invented calibration numbers.

## 9. Check the entire motion envelope and cable route

**Priority: high. Unverified combined system.**

The full intake has zero native motion joints, and its turret context is a static copy. The turret source has motion joints, including older V1 joints still present. Joint names or a healthy timeline do not prove that the active Mark 4 hardware, feeder, camera, encoder, intake, and chassis all move together correctly.

**Recommendation:** create one authoritative assembly with explicit fixed/yaw/pitch/compression groups. Verify physical stops and every allowed combination, using swept solids or sufficiently refined samples near minimum clearances. Model cable loops with connector and bend allowances; check powered-off stow/retrieval. Include ball and pressure-arm motion, not only rigid mechanism endpoints.

Avoid a slip ring until requirements justify it. A bounded yaw range with a well-managed cable loop may be simpler, provided it supports the shooting strategy and has physical constraints consistent with the rules.

## 10. Design for contact, stiffness, and repair

**Priority: elite reliability risk. Engineering judgment, not a stress calculation.**

The wide exposed sweepers, long pickup shaft, projecting motor hardware, printed bearing supports, curved rails, and printed pitch gearing deserve specific tests. The core concern is loss of alignment after load or contact. I did not establish material strengths, printed mass, center of gravity, shaft deflection, or an FEA safety factor from this model.

**Recommendation:** use replaceable intake modules, protected bearing/gear locations, positive shaft retention, and access to the final-feed jam. Prefer metal or appropriate plate construction for the most heavily loaded bearing/carrier supports, while using prints for guides and replaceable wear geometry. Retain the successful printed gearing until loaded tests show a reason to replace it. Provide removable contact liners and repeatable mechanical datums so a repair does not require complete recalibration.

Check pitch/hood displacement under representative ball force and after repeated direction reversals. Measure actual output rather than relying only on commanded servo position. Test sustained operation at the chosen supply voltage with realistic simultaneous loads.

## 11. Build legal acquisition behavior into the mouth

**Priority: game/integration risk; not a declaration that this robot is illegal.**

The wide mouth can encounter another ball while four are already staged. Include every ball under robot control in capacity accounting. Provide acquisition inhibition before taking a fifth. Distinguish opponent Nectar before intentional collection; a routine “collect then reject” strategy is not a substitute for avoiding prohibited control. Coordinate sensor placement with the width and visibility of the mouth.

Implement the TU05 rule checks: capacity ≤4 (G407), avoid opponent Nectar (G408), and enter FLOWER Nectar only in the final 60 seconds (G410). HIVE tips earn 20 points. [Official rules and scoring](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU05.htm).

**Recommendation:** prioritize a reliable HIVE cycle, then evaluate whether a simple FLOWER placement option provides enough late-match value for its time, packaging, and calibration cost. Lack of a dedicated FLOWER mechanism does not alone rule out elite performance; compare alliance strategies and measured results before adding one.

## What I would keep

- The stationary flywheel on a yaw frame with a lighter adjustable hood is a reasonable architecture to validate.
- Separate compression and pitch preserve useful tuning freedom for two ball sizes.
- A four-ball FIFO is a reasonable baseline. Sorting is not automatically worth its extra transfers and actuators.
- The centered turntable handoff is the right integration objective; its approach geometry needs correction.
- Purchased metal miter gears, bearings, thrust reactions, retainers, and cross-bracing already show attention to buildability.
- The printed gear/rack/rail success is real progress. Continue with a loaded prototype before changing those mechanisms solely for appearance.
- Limelight following yaw without pitch can simplify calibration, provided target visibility and the actual transform are verified.

## Recommended build order and decision gates

1. **Resolve packaging and the turntable collision together.** Place the real chassis and a candidate raised/reworked handoff, reserve wiring, and demonstrate stow and deployment. Keep this a candidate until the combined motion checks pass.
2. **Rigid bench shooter.** Establish repeatable same-size volleys into the real HIVE from a few locations, then characterize mixed compression and pitch. This decides whether the shooter deserves further integration.
3. **Short actual-tread transport module.** Prove the rise and one powered final-feed side, then a complete staged feeder. Do this before committing to every large printed panel.
4. **End-to-end four-ball loop.** Acquire, identify, stage, fire, verify exit, and recover from a jam. Test all size sequences and simultaneous arrivals. Measure total cycle time and interventions, not just muzzle interval.
5. **Integrated practice robot.** Add driving, autonomous repeatability, disturbance tests, sustained duty, and pit repair drills. Record distributions across many attempts, not only the fastest successful clip.

For deciding whether the design is approaching elite performance, track **successful HIVE tips per full match, successful acquisition-to-tip time, failed acquisitions/jams, automatic recovery time, autonomous repeatability, and performance after contact**. These are proposed development measures, not published world top-ten cutoffs. Field layout, alliance coordination, defense, driver practice, and available scoring elements affect actual match output.

A screening example: if a four-ball attempt requires all four balls to arrive and each independently succeeds with 95% probability, all-four arrival is only about 81.5%; at 99% it is 96.1%. This is illustrative reliability arithmetic, not a model of this HIVE's tip probability. It explains why improving consistency can beat reducing a nominal shot interval.

**My next investment would be a rigid shooter test stand and an adjustable transparent transfer rig, while fixing the shared packaging geometry.** The design does not need an exotic replacement turret to justify its next iteration. It needs evidence that its simplest complete scoring loop works repeatedly.
