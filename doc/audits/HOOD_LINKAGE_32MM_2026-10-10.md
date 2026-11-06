# Compression linkage revision — October 10, 2026

The repaired linkage is saved in Fusion as **Turret Hood Mark 3**, with a healthy 264-item timeline, and synced into **Ragnar V1 Turret DEVELOPMENT**, with a healthy 71-item timeline. This revision changes the purchased horn's attachment point and linkage articulation, not the carriage or other printed solids.

- Use the outer **32 mm** attachment hole on the modeled goBILDA **1900-0025-0104** H25T horn, replacing the former 16 mm attachment.
- Keep **58 mm between ball centers**, using the modeled **2913-0004-0241** female M4 steel ball links and 30 mm threaded rod. Confirm the actual owned ball-link type before assembling; other types may need a different rod length.
- The nominal 22 mm slider travel uses approximately **38.3 degrees** of horn rotation, compared with approximately 61.6 degrees previously. Absolute electrical servo positions require calibration; these are relative CAD angles.
- The rod angle relative to the guides varies approximately **20.0–39.6 degrees**, compared with approximately 36.2–59.4 degrees previously. The peak guide side load per unit axial load falls from approximately 1.69 to 0.83. This is an ideal geometric comparison, not a measured force or strength rating.
- The longer horn radius increases required servo torque for the same axial compression force. Measure compression force, holding current, temperature and carriage deflection before firing.
- Carriage, gray rails, orange shell, black insert, shaft holders and servo mounting solids were preserved. The carriage retains its existing plain Ø4.2 mm linkage eye. No new print is required for this linkage revision alone.
- The previous hidden stock linkage joints were still active after Fusion recomputation, creating a second constraint loop. The three obsolete joints were removed, while their old solids remain hidden. A complete pre-change F3D backup is retained.

Native verification drove the servo horn at all 23 positions from 0 to 22 mm. Actual endpoint travel was **21.9972 mm**; maximum discrepancy over the samples was **0.0270 mm** due to angular quantization. Cross-axis carriage displacement and pin separation were below numerical significance. Two yaw/pitch combinations preserved the relative linkage geometry. Temporary native-solid collision checks passed every 1 mm position over the entire stroke. These checks do not establish layer strength, fit, friction or servo capacity.

The V1 draft now has articulated horn and rod joints instead of rigid copies. Its 23-position native travel check also passed, along with integration collision checks at 0, 11 and 22 mm and two yaw/pitch combinations. The initial sync response timed out; the applied geometry was inspected and successfully verified/saved in a smaller continuation. The local V1 F3D was refreshed. This verification covers the compression linkage, not the full turret's pitch sweep.

The user's smooth hollow M4 standoffs can use their outside surfaces as guides without screws in the threaded ends. The current guide geometry assumes **6 mm outside diameter and 80 mm overall length**. A continuous smooth surface, sufficient length, secure external retention and physical sliding-fit testing are required. The guide bore is nominal Ø6.3 mm; it is not an M4 bolt hole. No shaft-holder changes were made in this revision.

Files:

- `tools/fusion/mcp_review/Turret_Hood_Mark3_LINKAGE_32mm.f3d`
- `tools/fusion/mcp_review/Turret_Hood_Mark3_before_linkage_32mm.f3d`
- `tools/fusion/mcp_review/Hood_Linkage_32mm.png`
- `tools/fusion/mcp_review/preflight_linkage_32mm_report.json`
- `tools/fusion/mcp_review/linkage_32mm_native_verification.json`
- `tools/fusion/mcp_review/linkage_32mm_v1_verification.json`
- `tools/fusion/mcp_review/Ragnar_V1_Turret_DEVELOPMENT.f3d`

The turret is still a development assembly. Pitch supports, servo mounting, sensor mounting and the approved 72 mm-wheel black insert revision are separate outstanding work; this linkage repair is not a release of the complete robot for printing or firing.
