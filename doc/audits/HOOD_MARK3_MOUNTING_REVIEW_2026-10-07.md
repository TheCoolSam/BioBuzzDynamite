# Turret Hood Mark 3 — mounting review

Reviewed the user's open Fusion design on October 7, 2026, using the already authorized local helper in `inspect` mode. No CAD geometry or joints were changed. Temporary visibility changes were restored. The neutral-pose snapshot is `tools/fusion/BioBuzzHoodTools/inspection_Mark3_2026-10-07.json`.

## Changes observed

- Both gray carriers and the yellow shell have revised mounting regions. The longer blue inboard supports now extend to Y=28 mm instead of Y=48 mm.
- Their paired Ø3.5 mm holes at Y=31 mm, Z=338/350 mm match the gray carrier and yellow shell holes. This gives a more definite bolt path and resists rotation better than the original single screw per support. Specify nuts/inserts and washer/tool access for that stack.
- The guide shafts remain Ø6 mm × 80 mm; the green carriage remains Ø6.3 mm at the sleeves. The black compression insert and green carriage solids are unchanged from the earlier concept.
- `2004-0025-0002:1` is imported at the design origin. [goBILDA identifies that SKU as Axon MAX MK2](https://www.gobilda.com/axon-max-servo-mk2/). The orange servo beside the linkage is still the provisional envelope, not this imported model.
- The imported servo has a 40×20 mm main case footprint and 54 mm overall flange span. Its outer Ø4 mm ear-slot cylindrical centers are at local X=±25 mm, Y=±8 mm (50×16 mm). Use these external mounting ears, not the smaller case-assembly screw locations. It has no native joint to the hood carrier yet.

## Carriage-to-insert issue inherited from the original concept

The insert has two Ø4.2 mm pilot sockets, centered at X=±18 mm and Z=338 mm along the Y direction. The carriage was trimmed against an insert solid that already contained these holes. Consequently, the carriage retained thin annular protrusions inside the sockets, around its Ø3.3 mm screw passages.

A native temporary-cylinder intersection probe from Y=67 to72 mm found **21.20575 mm³ of carriage material in each socket**. This is a real assembly obstruction for a heat-set insert occupying that region. It is an error in the earlier assistant-built attachment detail. It was identified, not repaired, during this review.

Recommended revision: add two integral bosses with flat outward-facing pads on the black insert's outer surface. Give the green carriage corresponding flat tabs, accessible screw-head seats and straight driver access. Heat-set inserts go into the black bosses before assembly; M3 screws pass from the outside through the green tabs into those inserts. Retain the smooth inner ball-contact surface. Match bore, depth and surrounding material to the actual inserts; the existing 4.2 mm pilots are provisional. [CNC Kitchen's design guidance](https://www.cnckitchen.com/blog/tips-and-tricks-for-heat-set-inserts) covers insert-specific bore dimensions, wall thickness and installation alignment.

## Other details to resolve

- Build a bracket supporting both mounting-ear ends of the actual servo, attached to the fixed pitching carrier or its guide supports. Align its spline axis with the crank axis. The servo body must not attach to the sliding carriage. Confirm pin/spacer and actual horn geometry before choosing bracket position.
- The two new blue-block holes are centered only 3 mm from the Y=28 mm end face. A Ø3.5 mm hole leaves about 1.25 mm of material toward that edge. This merits more edge material in a revision and a check of the existing print under bolt tightening; it is not a strength certification or an automatic demand to reprint the gray rail.
- The end-stop component is absent; `End_stops_follow_pitch` has an invalid occurrence reference. Rebuild/remove that obsolete joint when editing is authorized, and choose mechanical travel stops or indexing for the next assembly.
- Do not transfer the prior full-turret clearance results to Mark 3. This review did not drive the mechanism or repeat the assembled yaw/pitch interference sweep.
