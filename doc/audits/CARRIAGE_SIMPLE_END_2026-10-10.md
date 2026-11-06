# Plain rectangular carriage linkage end — 2026-10-10

User requested removing the awkward round linkage boss, retaining only a Ø4.2 hole, and an easier print.

Latest user model was at timeline 247 with five visible touching carriage bodies and no linkage-eye hole. User edits, including Split1, later extrudes and removal of body (4), were preserved. Hidden historical carriage body remained hidden.

The five visible carriage pieces were joined into the original `PRINT_Green_Carriage_flat_tabs` target body. A rectangular upright was added at X121.75..128.5, Y95..112, Z346..381 mm, with its front and back flush over the full upright height. A single Ø4.2 hole was cut along X at the original linkage pivot, Y101/Z375, through 6.75 mm. No outer circular boss remains at the eye. Minimum local edge ligament is 3.9 mm. Other component geometry was protected and unchanged.

Final Fusion document `Turret Hood Mark 3`: timeline 256, healthy, saved and unmodified after restoring assembly visibility. One visible connected solid. Volume 42043.17647126315 mm³. Bounds X-76..128.5, Y80..112, Z332..381 mm.

Native motion/collision check passed all 23 positions over 22 mm. Maximum solver error 0.017573 mm. Two Ø6 shaft probes, two M4 mounting probes, linkage eye probe, nut/socket and screw-head access passed. No interference with other local visible bodies. Shaft assumption remains nominal Ø6 ×80 mm. Hood mounting passages remain 32 mm.

Native and print-oriented STL topology passed: 116894 triangles each, closed, consistent winding, one connected shell. Print-oriented bounds 0..204.5 ×0..49 ×0..32 mm. Relative volume discrepancy from native under 0.0000032. Rigid transform places the flat rear face on the bed. Layer preview and physical load/fit testing remain required; no slicing or load rating claimed.

Final exports: `tools/fusion/mcp_review/prints/2026-10-10_simple_carriage/Carriage_Simple_End_PRINT_THIS_mm.stl`, native STL, CAD previews, notes and verification JSON. Package: `tools/fusion/mcp_review/prints/Simple_Carriage_2026-10-10.zip`.
