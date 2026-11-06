# Carriage strengthening — 2026-10-10

User authorized editing the carriage after accepting nominal 6 mm goBILDA round stainless guide shafting as the design assumption. Actual physical shaft identification remains unverified. User's two stop-tab removal cuts were already present at the start (timeline 213); those changes were preserved.

## Saved result

`Turret Hood Mark 3` saved in Fusion, final timeline 237, document unmodified after restoring display. Existing visible `PRINT_Green_Carriage_flat_tabs` body remains the same component, one solid. Final volume 40735.13176128843 mm³, bounds X -76..128.5, Y 80..112, Z 332..381 mm.

- Named Delete Face/heal removed the obsolete Ø3.3 blind pin passage and its end face.
- Rear crossbar additions increase nominal depth from 6 to 14 mm. They occupy X -76..-62.5, -51.5..51.5 and 62.5..121.75, with rail clearance reliefs between.
- Hood mounting pad reinforcement spans X ±18 ±8, Y 98..112, Z 332..348. M4 through passages remain Ø4.2, now 32 mm long rather than 25 mm.
- Broad triangular web, 14 mm deep, supports existing eye and joins toward the right guide. Six junction edges filleted with R2 mm.
- Small backing behind the rounded eye produces bed contact in the chosen orientation.
- Ø6.3 sliding bores, Ø4.2 eye passage and original pivot positions retained.
- Protected-body volume checks found no geometry changes to other parts, including gray rails, orange shell, black compression hood, blue supports, servo and stock linkage.

## Verification

Final native carriage tested against 26 other visible local bodies at every 1 mm position over 22 mm travel. No interference. Maximum motion solver error 0.017572630475200413 mm. Both shaft probes, two M4 hood mounting probes and eye probe clear. M4 nut and Ø9.5 socket access clear within tolerance. Hood screw head envelopes clear at both endpoints and mid-stroke. Healthy Fusion timeline; one connected solid.

Native and print-oriented STL files each have 118458 triangles, closed edges, consistent winding and one connected shell. Mesh bounds exactly match expected millimeter bounds to the report precision; relative volume difference is under 0.000003.

Print orientation is a rigid rotation/translation: `(x+76, z-332, 112-y)`. Envelope 204.5 × 49 × 32 mm; rear face placed at Z=0. No slicing or physical load test performed. Review bridges at rail reliefs and horizontal eye hole in Bambu Studio. PETG-CF, six walls, normal manufacturer profile suggested as first print.

Hood mounting passage length increased by 7 mm. Bolt length must accommodate actual insert engagement; a 40 mm screw plus 3 mm washer/spacer stack gives approximately 5 mm engagement. Existing black pilot cavity depth remains 6.7 mm. Do not bottom screws out.

Stock layout requires two nominal Ø6 ×80 mm smooth shafts. A purchased 100 mm shaft requires cutting to 80 mm. Printed fit and actual ball-load capacity still need physical verification.

## Files

`tools/fusion/mcp_review/prints/2026-10-10_strengthened_carriage/`

- `Carriage_Strengthened_PRINT_THIS_mm.stl`
- `Carriage_Strengthened_NATIVE_mm.stl`
- `PRINT_AND_ASSEMBLE.md`
- `Carriage_Strengthened_CAD.png`
- `Hood_With_Strengthened_Carriage.png`
- Native dimensions, CAD checks, mesh topology and dimension verification JSON.

Archive: `tools/fusion/mcp_review/prints/Strengthened_Carriage_2026-10-10.zip`.

Native scripts and responses: `strengthen_carriage`, `finish_carriage_print_face`, `verify_strengthened_carriage`, `save_export_strengthened_carriage`, `present_save_strengthened_carriage` under `tools/fusion/mcp_review/`.
