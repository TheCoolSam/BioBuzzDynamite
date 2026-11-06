# Ragnar: retain the shooter, finish its load paths and transfer

Research and design decisions — October 17, 2026

This is a proposed engineering revision plan, not a print release. No Fusion geometry or robot code was changed for this review. Evidence comes from the saved October 16 CAD inventory/clearance files, current geometry-generation sources, manufacturer documentation, and the team's new hardware and failure reports. It is not a fresh live inspection.

## Decision

Retain one flywheel shooter, geared pitch, the guided compression carriage, the existing printed gearing, a four-ball FIFO, and three intake/transfer/feed motors. Prioritize same-size bursts. Do not spend holding-force margin on a larger compression pinion simply to meet a mixed-ball 150 ms target that is no longer required.

Add a provision for a positive compression lock. Prototype the launcher with removable rigid compression blocks first. Correct the transfer collision and integrate the actual chassis before releasing large panels. Use the now-confirmed Tumbleweed tracks, with their actual tread width and articulation represented.

Kraken Pinion's reported failure was compression displacement under ball load, not yaw. That is valuable failure evidence, but it does not quantify our loads or prove that every cam or single shooter fails.

## 1. Compression: preferred implementation

The rack positions the carriage. A separate lock transfers firing force from the carriage into a bridge attached to both pitch rails. Locate the lock near the resultant load where packaging permits, to avoid introducing a large twisting moment. Guides continue to resist side loads and moments.

Use a straight metal indexing pin supported by a fork on both sides of a metal receiver: a double-shear load path. Provide two independently adjustable receiver positions for the two calibrated ball settings. Use positively located/clamped receiver blocks, replaceable metal wear surfaces, and through-bolted attachment to the carriage. Printed housings can locate parts; thin printed tabs and printed detent teeth should not be the sole impact restraint. Final pin diameter, plate thickness and edge distances require measured load and a strength check.

Prefer a spring-engaged pin with a small existing standard servo retracting it. The spring inserts the pin; the metal shank and receiver carry transverse shooting load. This does not require another DC motor. A full pin-engagement sensor is preferable to assuming a servo command succeeded. A rounded spring-ball detent that cams out under force is not an equivalent lock. Carr Lane explicitly distinguishes positive locking pins from ordinary detent pins [S4]. This is a design principle, not a claim that a particular catalog ball-lock pin fits our CAD.

Switching sequence: confirm the preceding ball has cleared and stop the feed; use the rack to relieve pin side load; retract and verify the pin; move the carriage; engage and verify the pin; gently seat the carriage consistently on the locating surface; allow settling; permit feeding. Test release under realistic residual preload. The lock must never need to retract while a ball is applying its shooting load. Avoid sustained servo stall against the locator. Add a manual pin/locking-block fallback using the same interfaces.

Clearance in a pin hole still allows motion. Use consistent preload against a locating face, inspect local plate/receiver flex, and measure the ball-contact surface itself. Internal servo feedback alone cannot reveal rack backlash, carriage bending, or a loose receiver. A lock prevents back-driving; it does not automatically make the hood rigid.

Keep the present 24T MOD0.8 pinion and 9 mm printed rack for the first build. Neither spur nor herringbone gearing is inherently self-locking. The rack's nominal pitch radius is 9.6 mm. Ideal reflected torque is T = F_parallel × 0.0096 m. Illustrative axial loads of 25, 50 and 100 N give 0.24, 0.48 and 0.96 N·m respectively. These are sensitivity examples, not measured ball loads. Use the component of ball load along carriage travel; evaluate moments and side forces separately. Published servo stall torque is not a continuous-duty or impact stiffness rating [S2].

Alternatives considered:

- Servo holding alone: least added hardware; useful as a comparison test, but depends on supply, backlash, compliance and transient control response. Do not declare it inadequate solely from another team's cam failure.
- Positive pin lock: preferred modification because it retains our rack and carriage and directly restrains translation. Costs a release mechanism, engagement detection and a tested unloading sequence; gives two discrete shot settings.
- Over-center toggle clamp: mechanically credible; DESTACO uses an over-center linkage for holding [S3]. A viable backup if a pin cannot package, but a homebuilt two-setting version needs geometry, overtravel, anti-flex stops and vibration/release checks. One toggle does not automatically lock both ends of a 20 mm stroke.
- Dwell cam: a properly profiled dwell can reduce load-induced input torque. Ideal virtual work gives T = F × dx/dtheta. A dwell has dx/dtheta near zero; a ramp does not. This explains why not all cams behave like the reported failed mechanism. Replacing the proven-fit rack with a new cam adds development work now.
- Lead screw: appropriate low-lead screw/nut pairs can resist back-driving, but material, friction and vibration matter [S5]. At a 2 mm lead, 20.32 mm requires 10.16 revolutions; our MAX MK2 has a specified maximum 360° positional range [S2]. A direct screw substitution therefore does not fit the present actuator/travel. A high-lead screw sacrifices the self-locking advantage. Do not casually replace the drive with a worm either; reduction, range, backlash and actual self-locking all need redesign.
- Passive spring compression: helpful for acquisition, but changes hood position with ball force. Keep the shooting compression positively located while leaving compliance in the tires and entry pressure rollers.

The nominal 20.32 mm stroke is the diameter difference, not a demonstrated optimum. Tune each gap independently with real new/worn balls. Preserve adjustment range around both settings; do not drill irreversible fixed indexing holes at nominal values before shooting tests.

## 2. Transfer and packaging: one shared problem

The audit's second parked Nectar intersects four turntable bodies. Its center is (0, -78.5014, 80.72) mm. This is a confirmed hard-structure conflict. Existing 30/35 mm table-lift checks clear four parked spheres only; neither is a validated moving transfer.

First candidate: reshape the fixed rise so the ball is sufficiently centered under the bore before it crosses the plate. Keep a round, coaxial stationary-to-yaw handoff with no fastener, lip or offset narrowing the available passage. Make transitions flush in the travel direction and leave jam access. Do not assume the existing guide's imposed centerline is where an unconstrained ball will actually go.

A useful geometry screen: for a nominal 139.7 mm bore and 91.44 mm sphere, when the sphere's equator crosses a plate plane its center must be within (139.7 − 91.44)/2 = 24.13 mm of the bore axis, before allowance for liner thickness and clearance [S6]. At other heights, evaluate the actual sphere cross-section and every plate's real opening. This illustrates the centering requirement; it does not certify the noncircular CAD openings.

If a revised rise cannot fit and retain continuous driven contact, use the minimum relative table lift that passes the full transfer sweep. Keep the hood-to-turntable supports short. A small table lift relative to the magazine differs from adding tall hood uprights. Compare the two candidates in the complete chassis, rather than picking the lowest table independent of storage.

Current folded visible solids span 452 × 456.26 × 387.99 mm at the modeled 115° fold; full chassis, wiring and connector allowances are missing. Start with a proposed complete stowed target around 440 × 440 mm horizontally, with floor-referenced height checked separately. This is a margin target, not a rule or a verified fit. Recover roughly 12 mm width and 16.3 mm length from the present solids, then include omitted hardware. Moving side pods inward and shortening the fixed landing section are candidates, not clearance-approved edits. Preserve useful intake width where possible.

Four nominal Nectar with three 4 mm gaps occupy 377.76 mm along a straight line. Do not shorten the magazine merely to hit a box dimension without rechecking all ball sequences and curved spacing. Place drive modules on both sides of the central magazine, reserve battery/hubs low where possible, and include all three intake drives, flywheel drive, encoder and service access.

FIRST TU05 requires an 18-inch starting cube; the operating envelope is 18 × 24 × 29 inches, with physical expansion constraints [S7]. Turret yaw can change the horizontal footprint, so isolated deployed length does not establish operating fit.

## 3. Tumbleweed: a newly identified CAD mismatch

Confirmed hardware: goBILDA 2409-0001-0001 Tumbleweed, 32 mm wide, 8 mm chain pitch, 50A tread [S1]. The source currently generates two 24 mm-wide swept track envelopes, centered at X = ±22 mm, and places side guides from local X = +14 to +18 mm relative to each track center. The inventory confirms those 24 mm widths.

Real tread is 4 mm wider on each side. Its lateral projection reaches 2 mm into the current guide-plate region. This is a new interference risk identified from envelope/source comparison, not a completed OEM BRep collision check; tread height and chain profile determine actual contact. Replace the envelope with actual link/tread geometry and move or recess guides to support the chain without rubbing tread. Check every sprocket wrap and adjacent fastener.

Pitch compatibility does not establish reverse articulation, link clearance, or bend radius. Manufacturer data reviewed do not certify our proposed concave loaded rise. Check actual links by hand, then in a short clear-sided powered module. Use real link count and a tensioner: 104 chain pitches is 832 mm, not automatically one 800 mm purchased assembly. Count pitches rather than confusing a treaded segment with one 8 mm link; the product describes segments assembled from different chain links [S1].

Retain independent passive pressure stations with adjustable spring anchors and positive travel limits. Select preload by measured feed force and slip, not tire durometer alone. Large balls should open the arms without reaching a hard pinch; small balls must keep driven contact between stations.

Preferred powered layout: motor 1 drives pickup and both horizontal sweepers; motor 2 drives the magazine and any fixed rise/transfer rollers; motor 3 drives the yaw-mounted final feeder. Keep DC allocation at three for this subsystem. Do not demand that passive rollers carry balls through a section where the tread has lost contact.

Retain the curved tread route only if the real track passes articulation, drag and loaded transport tests. Otherwise keep Tumbleweed on a conventional straight/sloped loop and add overlapping powered transfer rollers driven from motor 2 into the bore. That fallback changes the transfer module, not the whole robot; capacity, packaging and transmission still require CAD checks. The fixed-to-yaw handoff should stay coaxial and unobstructed.

## 4. Final feeder and staging

The existing short moving feeder carriers, purchased metal gears, bearings, thrust provisions and stops are already present. Do not replace them based solely on visual complexity. Bench a complete powered side, measure carrier opening under drive torque and tune spring leverage/preload. Forward drive must not unload contact enough to slip; reverse must not wedge or damage balls. Reorient the drive or compare a belt-driven carrier only if measurements expose a problem. Extra spring preload is not a universal cure.

Separate magazine motion from final release. Establish a retained staging position below the shooting contact arc, an actual stage detector, and an exit confirmation detector outside wheel/ball collision paths. Include sensor brackets, opaque/reflective ball behavior, wire routing and response time. Roller braking alone is not an established escapement. Test queue pressure and coast-down; add a compact positive gate if the stopped feeder cannot reliably retain the next ball. A gate or latch servo can use the team's standard servos without consuming another DC motor, but its electrical budget still counts.

Stopping a floor tread under freely spinning upper rollers may not hold a rolling ball. Staging must prove retention mechanically, not assume belt-stop equals ball-stop. Keep an accessible reverse path and removable transfer cover for jam recovery.

## 5. Close every remaining audit item

Audit 1 — transfer collision: center the rising ball in the real bore or apply the minimum fully checked relative lift. Pass moving ball/structure checks, not only parked positions.

Audit 2 — sizing: one master assembly with real drivebase, floor plane, cables, fasteners, connectors and mechanically defined stow/deploy stops. Check intermediate fold and all allowed turret yaw/pitch combinations. Target manufacturing margin.

Audit 3 — curved rise: actual 32 mm Tumbleweed geometry, real articulation/tension, adjustable pressure stations and powered contact through every transfer.

Audit 4 — final carrier: retain modeled hardware; bench drive-reaction opening, spring force, reverse behavior and shaft retention. Compare a simpler drive only on evidence.

Audit 5 — staging: one commanded release/one detected ball; following ball remains outside flywheel contact despite queue force and coast. Model actual sensors and any required gate.

Audit 6 — mixed compression: deprioritize 150 ms switching. Retain current pinion and optimize same-size volleys. A full ideal size change is about 0.202 s at Axon's published 7.4 V speed, before locking/settling [S2]. Use the measured supply and loaded timings.

Audit 7 — shooter/HIVE: fixed compression and pitch first, then automatic compression, pitch, bursts and complete HIVE trials. Measure speed recovery, actual gap motion, hit scatter and tip repeatability. Three/four-ball tips remain experimental results to establish.

Audit 8 — actuators/power: confirm the full 5203 SKU/RPM; place flywheel motor, transmission, shaft retention, tension adjustment and guard. Finish bounded yaw actuation and a real output-coupled encoder transmission. Locate stationary brackets without blocking robot attachment. Include servo connectors and supply current; manufacturer maximum voltage is not the voltage supplied by our hardware. REV's current Servo Hub page specifies 5–7.4 V output, 6 V default and 15 A total shared [S8]; verify the team's actual device and configuration. No code calibration values are invented or enabled here.

Audit 9 — integrated motion: replace the static-only context workflow with one authoritative articulated assembly; define fixed, yaw, pitch, compression, pickup float/stow and feeder motion groups. Suppress legacy joints from active validation. Include balls, pressure arms, cables and connector bend allowances, plus camera line of sight.

Audit 10 — repair/stiffness: retain the corrected pitch-servo cradle/frame interface and accessible bearing retainers. Use metal where shaft, lock or bearing loads concentrate; prints for shapes, spacers and replaceable guides. Prefer removable wear liners, captured nuts/through-bolts, positive shaft retention and consistent replacement datums. Verify printing orientation/support removal for every released part. Gear fit success is not a loaded durability certificate.

Audit 11 — acquisition: model sensing before the committing nip where practical, protect against an accidental fifth controlled ball, and provide capacity inhibition. Opponent Nectar avoidance must happen before intentional control; do not rely on collect-then-reject as the strategy. TU05 G407/G408 limit capacity/control, and G410 restricts entering Nectar into Flower scoring volume to the last 60 seconds [S9]. Keep the FIFO and avoid adding a sorter. Same-size targeting is an acquisition strategy, not an automatic reorder capability.

## 6. Build and test order

1. Build the present shooter on a rigid stand with fixed pitch and removable compression blocks. Measure gap on both sides and record motion during contact, not just its resting return. Use several real balls of each size and condition. Characterize Nectar and Pollen separately.
2. Sweep useful compression gaps at a fixed range/RPM, then RPM at the selected gap. Do not assume the two nominal diameter-derived settings are optimal. Record shot scatter as well as mean impact point.
3. Fit the positive lock and compare fixed-block, locked and servo-only shots at the same conditions. Verify release/re-engagement and output gap after repeated switching; tune pin clearance and load-bearing stiffness. Do not accept a lock that only engages intermittently.
4. Test same-size 3/4-ball volleys over several release intervals with the complete real HIVE. Log RPM before/after each ball, actual exit interval, impact point and tip result. Determine whether recovery, feed or impact dynamics sets cadence. Keep a useful reliable setting even if the fastest volley fails.
5. Build the short actual-tread transfer rig and one complete powered feeder side while the shared chassis/transfer CAD is corrected. Test held queues, starting, stopping, reversing, worn balls and side-by-side arrivals.
6. Test the full four-ball path in all 16 P/N orderings. Begin with small repeatable trial sets, then run at least 100 cycles on the difficult cases as a screening target. Record every intervention and damage; zero failures in a short test is not proof of match reliability.
7. Integrate driving, voltage sag, contact, cable motion, camera visibility and repair/reassembly. Recheck shot calibration after changing mechanical geometry or tires.

Suggested initial measurement targets, explicitly provisional: compression position repeatability within 0.25 mm unloaded and transient opening below 0.5 mm at the ball-contact face. Measure left/right gap and tilt. These are starting engineering targets, not known scoring thresholds; tighten or relax only after relating displacement to measured shot scatter. Size the lock and supports from measured representative load with a documented design margin. Hand-push force tests alone do not capture the shooting impulse.

Release small fit coupons, the corrected rack/gear parts already proven to mesh, and bench-fixture parts as appropriate. Hold large conveyor/chassis panels until the 32 mm track, hard-structure transfer and full packaging checks pass. Preserve the existing successful printed parts wherever their interfaces remain valid.

## Sources and evidence

- S1: [goBILDA Tumbleweed product](https://www.gobilda.com/tumbleweed-tank-track-800mm-length-50a-durometer-treads//), [track family width/pitch](https://www.gobilda.com/tracks), [manufacturer spec sheet](https://www.gobilda.com/content/spec_sheets/2409-0001-0001_spec-sheet.pdf).
- S2: [Axon MAX MK2 official specifications](https://docs.axon-robotics.com/servos/max).
- S3: [DESTACO explanation of over-center toggle holding](https://www.destaco.com/resources/blogs-and-insights/2000-series-hold-down-action-manual-toggle-clamps).
- S4: [Carr Lane positive locking pins](https://www.carrlane.com/product/ball-lock-pins-quick-release-pins/ball-lock-pins), [technical guide distinguishing detent pins](https://www.carrlane.com/Portals/0/PDFs/CLM-Trig%20Booklet-ENG-v4-PM.pdf?timestamp=1664294001935&ver=N97ZnmVVMsM7Xc5Fxkcnpw%3D%3D).
- S5: [igus lead screw technical data: self-locking and high-helix differences](https://www.igus.com/linear-bearings/lead-screw-units-trapezoid-spindle-technical-data), [Thomson back-driving and brakes](https://www.thomsonlinear.com/en/support/tips/is-it-possible-to-backdrive-an-actuator).
- S6: [AndyMark Robits 5.5-inch ID assembly](https://andymark.com/products/robits-5-5-in-turntable-assembly).
- S7: [FIRST TU05 Section 12, R102/R105](https://ftc-resources.firstinspires.org/ftc/game/manual-12).
- S8: [REV Servo Hub official product specification](https://www.revrobotics.com/rev-11-1855/).
- S9: [FIRST TU05 Section 11, G407/G408/G410](https://ftc-resources.firstinspires.org/ftc/game/manual-11).
- Saved CAD evidence: `tools/fusion/season_review_2026_10_16/inventory.json`, `clearances.json`, `motion_metadata.response.json`; `tools/fusion/mcp_review/RAGNAR_INTAKE_V2/oct16_live_checkpoint.json`, `oct16_verification_summary.json`, `oct16_rack_revision.json`.
- Geometry source: `tools/fusion/mcp_review/intake_v2_geometry.py` (24 mm track envelopes and guide offsets), `intake_v2_math.py` (queue assumptions), `intake_v2_roller_feeder.py` (existing driven carriers).

Sources checked October 17, 2026. Manufacturer sources establish hardware properties and mechanism principles; the proposed lock/transfer design and provisional acceptance targets are engineering recommendations, not manufacturer endorsements or completed CAD validation.
