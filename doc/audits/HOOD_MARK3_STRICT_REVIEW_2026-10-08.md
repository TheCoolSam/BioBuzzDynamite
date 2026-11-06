# Hood Mark 3 strict review — October 8, 2026

**Verdict: start a fit build tomorrow; do not print every part as final.**
The black compression piece is ready for a rigid-material fit print after
testing the insert coupon. Revised yellow and blue parts are suitable for a
fit build. Hold final green carriage and 80T ring prints until their physical
connections are finished. Gray rails can be printed/drilled for the current
layout, but the gear attachment may require additional holes.

## Fixed in Fusion

- Replaced the rejected printed servo bridge with manufacturer steel brackets,
  a custom 3 mm aluminum angle and a 2 mm aluminum backing plate. The assembly
  is rigidly attached to the gray pitching carrier and therefore follows its
  pitch and yaw. Original imported servo and original solids remain available.
- Added right blue clearance, yellow backing/nut clearance and gray mounting
  holes. Servo force does not rely on a printed bridge or blue clamping stack.
- Reinforced both rear supports and moved their two fastening holes away from
  the shaft. The old shaft-to-fastener bore ligament was approximately 0.065 mm.
  New upper M3 bolt-to-shaft envelope clearance is 6.5 mm.
- Suppressed obsolete servo/link joints and the missing end-stop joint. Rebuilt
  mispositioned imported internal hardware from its recorded original transforms;
  those physical envelopes are included in the final interference check.
- Exported native metal-face DXFs and a rail drilling template. Prepared a
  223 × 223 mm flat rail orientation for a 256 mm bed.

## Findings that still block a final mechanism

| Severity | Finding | Required action |
|---|---|---|
| High | Green drive-pin Ø3.3 bore is blind, 18 mm deep, with no demonstrated retention; it cannot simply accept a threaded M3 screw | Finish retained metal pin/bolt geometry and select horn/link hardware before final green print |
| High | Real horn plane is farther outboard than the old hidden linkage; the imported servo spline reaches about X=124 while green ends at X=117 | Model actual horn, pin/spacers and link; check motion and side loads |
| High | 80T ring is separated 4 mm from the gray carrier and has no bolt holes | Design actual torque-transfer connection/spacers; rigid CAD joint is insufficient |
| High before firing | No physical firing/travel stops or verified shaft retention | Fit actual hardware, set mechanical travel and calibrate servo commands |
| Medium | Ø6.3 green bores on Ø6 shafts leave only 0.15 mm nominal radial clearance | Test actual print/shaft fit and parallelism with all mounting bolts tight |
| Medium | Mounting stiffness and print strength have no measured proof | Hand-load the assembled mount; then test under representative ball force and check servo current/temperature |
| Medium | Front blue/gray/yellow fasteners need full-stack lengths and an assembly sequence | Start with M3×40; verify washers/nuts/thread engagement and access before assembling black insert |

The yellow hood has four aligned front mounting holes through its side tabs;
its two older Ø4.2 holes are a different pattern. The revised metal mount needs
a pocket in the yellow part; use the revised yellow export. An already printed
gray rail can receive the new rear and servo holes using the drilling template.

## Evidence and limits

Final native BRep intersection checks found zero local neutral interferences
and zero slider interferences at twelve positions from 0 to 22 mm, spaced
2 mm apart. Four Ø4 ear-bolt probes clear the servo; each has about 26.75 mm²
of ear material under an 8 mm washer envelope. Mount bolt locations have full
10 mm washer support on the gray carrier; both rear locations have full 8 mm
washer support. These checks include corrected imported servo hardware.

All delivered STL variants pass closed-edge, consistent-winding, connected-shell
and positive-volume checks. Each revised native print body is one solid.
The black contact geometry is unchanged. Native source save completed successfully.

This is not a tolerance simulation, finite-element strength analysis, continuous
motion proof, ball-compression validation or shot-performance test. The full
turret was not re-audited. Physical hardware is necessary to clear final assembly.

Build instructions and the print decision table are in
`tools/fusion/BioBuzzHoodTools/prints/2026-10-08/metal_mount/PRINT_AND_BUILD.md`.
Evidence files there: `validation.json`, `final_native_validation.json`,
`pose_validation.json`, and `fit_prints/mesh_validation.json`.
