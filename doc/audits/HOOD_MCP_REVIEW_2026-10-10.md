# Hood review through Fusion MCP — October 10, 2026

**Subsequent authorized change, October 10:** the two black compression insert
pilots were changed from Ø6.0 to Ø5.8 mm by editing the existing `OffsetFaces1`
distance from −1.0 to −0.9 mm. Both remain 6.7 mm deep. Native post-verification
confirmed a single solid, 176 healthy timeline entries, and unchanged volumes
for the other occurrence bodies. The cloud document was saved; a separate
read-only check confirmed the saved dimensions and an unmodified document.
The hole table below records the original review snapshot.

**Approved rear/linkage revision completed and saved, October 10:** both gray
rails now have matching Ø4.2 mm rear passages at Y=124.978719 mm, Z=324/333 mm.
The gray rails share one component; exactly 221.670778 mm³ was removed from its
original visible body, corresponding to two Ø4.2 x 8 mm holes. The orange shell
is unchanged. The user's already printed rails will be left untouched and
replaced by reprints.

The existing green carriage now has a joined, gusseted eye at Y=101, Z=375 mm,
with a short Ø4.2 mm transverse bolt passage and a 6.75 mm thick round boss.
Stock linkage references use a goBILDA 1900-0025-0104 H25T horn at its 16 mm hole,
two 2913-0004-0241 steel ball links, a 30 mm M4 threaded rod, and 58 mm between
pivots. A nominal 1 mm spacer is required between each pivot ball and its
mounting face. The reference assumes approximately 10.37 mm rod engagement
per end, leaving approximately 9.26 mm between the threaded mouths for jam
nuts. The user has a visually matching horn but has not identified the ball
links; different end fittings may require another rod length or spacers.

The right blue front/rear supports gained integrated stop lugs with Ø4.2 mm
axial holes. Fully threaded M4 stop screws and retaining nuts are required;
no screw, nut or washer models were added. At nominal 0/22 mm travel, the stop
head contact planes are Y=104/76 mm. Their projections must be adjusted during
assembly. The two existing rear cap bodies became separate components with
rigid joints to their guides; their shapes and initial poses are unchanged.
The two custom shelf holes were normalized from Ø4.3 to Ø4.2 mm. Purchased
servo/bracket geometry was not altered.

Native horn-driven motion was measured at every millimetre from 0 to 22 mm.
The new mechanism retains connected pivots; solver stroke differences were
less than 0.03 mm. After the final stop/gusset corrections, solid interference
checks at 0/11/22 mm found no conflicts among 27 visible local bodies. Seven
Ø4 mm bolt probes passed through both rear stacks, the green eye, and both
stop lugs. Forty-three checked custom M4 cylindrical bores measured Ø4.2 mm.
Two pitch/yaw changes preserved local relative geometry within numerical
precision. All 211 timeline entries recomputed without health issues. The
design was saved and remained unmodified after STL export and presentation.

STLs for the two-copy gray rail, green carriage, right front/rear blue supports,
and updated black compression piece are in
`tools/fusion/mcp_review/prints/2026-10-10_linkage_revision`. All five meshes are
closed, consistently wound, single-shell solids. Native and mesh dimensions
are recorded separately. These checks establish CAD fit; physical insert fit,
slider friction, stop adjustment and load strength still need assembly tests.

**Verdict: the local fit geometry passes, but the complete hood mechanism is not ready for a final build.**

Reviewed the live, unsaved `Turret Hood Mark 3` document through
`http://127.0.0.1:27182/mcp`, using enforced read-only scripts. The timeline
contained 176 entries and had no reported health errors or warnings. The
geometry and timeline were unchanged by the measurements. Screenshot capture
changed the viewing direction only. The full turret and shot performance were
outside this review.

**What passes**

- All 19 visible local mechanical bodies are single-lump solids. Native
  temporary-solid intersection tests found no overlaps at the current pose.
- The black insert and green carriage clear the fixed visible hood geometry
  at twelve sampled positions: 0, 2, 4, …, 22 mm inward. This tests discrete
  positions, rather than continuous motion or printer tolerances.
- The carriage's active slider joint has limits of −22 to 0 mm. The guides,
  gray rails, yellow shell, servo mount and actual servo remain connected by
  healthy rigid joints to the pitching assembly. The black insert is rigidly
  joined to the carriage.
- The front fastener pattern lines up at hood-local Y=31 mm, Z=338/350 mm.
  The right stack contains 15 mm of blue support, 3 mm of mounting angle,
  8 mm of gray rail and 5 mm of yellow attachment tab: 31 mm total. The left
  stack also totals 31 mm. Four-millimetre bolt probes pass through both gray
  rails at the front positions. The upper angle hole differs from the other
  centers by approximately 0.010 mm, within its nominal geometric clearance.
- Your new rear cap bodies have matching holes in the blue supports. Their
  placement covers the rear ends of the guide-shaft bores.

**Hole dimensions checked against your latest instructions**

You specified heat-set inserts with a 5.8 mm recommended pilot and described
the screw holes as 4 mm.

| Location | Current CAD dimension | Assessment |
|---|---|---|
| Two black insert mounting bores | Ø6.0 mm, 6.7 mm deep | CAD differs from the specified Ø5.8 mm pilot, but Ø6.0 may be valid compensation for undersized printed holes. Test Ø5.8/5.9/6.0 coupons using the actual inserts, material, orientation and slicer settings before selecting a final CAD diameter. Confirm insert length also fits. |
| Two green-to-black bolt passages | Ø4.2 mm, 25 mm long | Aligned with the black bosses; nominal clearance for a 4 mm bolt. Select bolt length from the actual insert engagement and washers. |
| Yellow, visible gray, blue support and rear-cap screw passages | Ø4.2 mm | Larger than an exact Ø4.0 mm hole; the extra diameter is clearance, not a leftover M3 bore. |
| Large angle's two upright mounting passages | Ø4.2 mm | Align with the front hood pattern. |
| Large angle's two shelf mounting passages | Ø4.3 mm | Still differ from the other custom screw passages. |
| Shaft passages | Front supports Ø6.2; rear supports Ø6.4; carriage Ø6.5 mm | Intentional shaft holes, not screw holes. Carriage clearance on a Ø6 shaft is 0.25 mm radially; test both sliding and play. |

Imported servo/bracket geometry contains thread and screw-detail cylinders;
those were not treated as custom clearance holes. A Ø4.0 mm CAD hole has no
nominal clearance around a Ø4.0 mm screw, so do not shrink every clearance
passage solely to make the numerical values identical. Validate printed fit
with your actual hardware.

**Assembly issues to finish**

1. **Rear supports cannot yet bolt to the visible gray rails.** Both blue
   supports have transverse holes at Y=124.978719 mm, Z=324 and 333 mm. The
   corresponding regions in both visible gray `Body1` rails are solid.
   Four Ø4 mm bolt probes were completely blocked by rail material. Add the
   matching rail holes, or define a different physical rear attachment.

2. **The servo linkage is absent.** The old horn and connecting rod are hidden
   and their three motion joints are suppressed. The visible green drive arm
   has no retained link-pin hole. Finish the actual horn, rod and carriage pin
   connection before treating the green print as final. A collision-free
   slider does not establish servo-driven motion.

3. **The gear has no demonstrated physical connection to the gray carrier.**
   Its nearest surface remains 4.0 mm from the right gray rail; the gear occupies
   X=65–70 mm while the rail ends at X=61 mm. No mounting holes are present in
   the gear. The active rigid CAD joint makes it move with the rail, but a
   physical torque-transfer connection and axial retention are still required.

4. **The shaft caps need assembly ownership and fastening.** Root `Body2` and
   `Body3` are currently not visible according to the API, although their own
   lightbulbs are on. They have no component-level assembly joints. Put each
   in the relevant guide component, or give separate cap components rigid
   joints to the guides. Their Ø4.2 mm holes are clearance passages, so define
   nuts or another actual retention method rather than assuming screws thread
   into them.

5. **The 22 mm limits are software/CAD limits, not physical firing stops.**
   No adjustable stop components are present. At either commanded extreme,
   the relevant carriage sleeve remains 8 mm from its blue support face.
   Finish mechanical stops if firing pressure is intended to seat the carriage
   against a fixed position rather than load the servo.

6. **The large shelf still needs a fabrication decision.** The visible
   `ALUMINUM_3mm_angle_DO_NOT_PRINT` is a custom body, currently spanning
   50 × 66 × 42 mm overall. The two small steel brackets are purchased parts;
   the shelf is not a verified stock part. If avoiding machining remains a
   requirement, select an available support and check that exact part, or
   explicitly redesign and validate a printed shelf for the intended loads.

The hidden fitted servo hardware was also checked at the current pose. It
does not overlap external hood parts; reported intersections are within the
servo's own screw/case interfaces. That check does not validate unmodeled
assembly bolts, a future horn, or a future linkage.

**Print decision**

Test the black insert fit before the final print; a Ø6.0 mm CAD pilot is not
automatically a defect. Its actual printed size and insert retention determine
whether the extra 0.2 mm is appropriate. Match the hole depth to the actual
insert length as well. Hold final gray
rails until rear attachment and gear attachment are resolved. The green
carriage needs its drive connection before a final print. Blue pieces and
caps can be used for fit prototypes, with the above attachment and clearance
checks. No new STL exports were made; previous exports do not contain all of
your current edits.

This review establishes native geometry and joint state. It does not establish
printed strength, friction, backlash, servo loading, ball compression or firing
repeatability. Those require the assembled hardware and physical testing.

Evidence is retained in `tools/fusion/mcp_review/current_model.json`,
`current_geometry.json`, and `verify_connections.response.json`, with the
read-only request scripts and native MCP screenshots alongside them.

Pilot-sizing clarification: [CNC Kitchen's direct tests](https://www.cnckitchen.com/blog/are-our-heat-set-insert-datasheets-wrong)
show why CAD enlargement can compensate for printed undersizing. Their test
values are specific to their inserts and print setup, rather than a universal
offset. [SPIROL's hole-design guidance](https://www.spirol.com/resources/white-papers/how-to-design-the-proper-hole-for-heat-ultrasonic-inserts/)
explains the retention penalty from an excessively large actual hole.
