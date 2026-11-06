# Ragnar Mark 4: compression concept and tipping model audit

## Saved CAD deliverable

`Ragnar Hood Mark 4` now has 30 healthy timeline items, with a separate
sideways selector and pitch joint. Mark 3 and Ragnar V1 remain unchanged.
The local archive is `tools/fusion/mcp_review/Ragnar_Hood_Mark4_SELECTOR_CONCEPT.f3d`.

This is a motion demonstration, not a production design or print release.
The original native build recompute reset the trial poses; a follow-up verified
the actual occurrence transforms without recompute and corrected the images
and report. Final state is neutral pitch, pollen selected.

- A fixed nectar contact surface has radius 123.44 mm about the wheel axis.
- A smaller pollen liner has radius 103.12 mm. It slides along the wheel shaft
  axis, not toward the wheel or upward. It moves 78 mm to park for nectar.
- Both surfaces are concentric with the 72 mm contact wheel. Nominal gaps are
  87.44 mm and 67.12 mm. Each is 4 mm below its nominal ball diameter; this is
  a trial value only and is not a validated operating compression.
- Both profiles and guides follow hood pitch. The wheel reference stays fixed.
- In the concept, guide bores are 6.3 mm, assumed rods are 6 mm, and new M4
  mounting holes are 4.2 mm. Physical standoff diameter is still unconfirmed.
- Temporary BRep checks at fourteen selector positions found no liner overlap
  with the concept frame or guides. The parked liner clears sampled centered
  nominal nectar spheres by at least 3.28 mm laterally. Actual joint transforms
  were checked at six pitch/selector combinations.

### Unfinished mechanical details

The large frame is an illustrative support envelope, not a proposed one-piece
print or an actual purchased metal part. Servo transmission, guide retention,
pitch support/drive, ball feed, entry/exit transitions, shallow transverse
centering, stiffness and full robot packaging remain to be engineered.
The liner extends past the fixed shoe's arc to reach its guide supports;
therefore matching final ball exit positions/angles is NOT demonstrated.
The demo's -55 to 0 degree joint range is NOT the chosen shooting range.
User's steep trajectories need a suitable larger useful launch range.
The radius parameters from the earlier layout sketch do not drive these
base-feature bodies. Changing parameters alone does not resize this concept.

## Audit of the supplied impulse estimates

Source nominal masses: [AndyMark scoring elements](https://andymark.com/products/biobuzz-scoring-elements).
Pollen 0.055 lb = 0.02494758035 kg; nectar 0.091 lb = 0.04127690567 kg.

The team's estimated I = 0.18–0.35 kg m² and effective barrier = 0.20–0.30 J
give L_required = sqrt(2 I barrier) = 0.2683–0.4583 kg m²/s.

For ideal capture transfer eta=1, r=0.50–0.65 m, initially stationary hive,
constant effective loaded inertia and coherent instantaneous impacts:

- 3 nectar need useful perpendicular impact speed 3.3337–7.4014 m/s.
  At the proposed 3.1 m/s they provide only 0.1919–0.2495 kg m²/s,
  below the entire proposed requirement interval.
- 4 pollen need 4.1368–9.1844 m/s. At 4.2 m/s they provide
  0.2096–0.2724 kg m²/s, enough only at the favorable corner.
- At I=.265, barrier=.25, r=.575, eta=.8, the requirements are
  6.3903 m/s for 3 nectar and 7.9298 m/s for 4 pollen.

These are conditional calculations, not measured hive thresholds. The
parameter ranges, particularly lever arm and energy barrier, must be checked
on the actual field. Eta=1 is an ideal capture assumption, not a bound on
instantaneous impulse against a rebounding ball. A retained ball also changes
inertia and gravitational loading. Real separated impacts require a time
domain model of hive motion, load, friction and damping.

## Trajectory consistency

The supplied vacuum trajectories are internally consistent:

- Nectar: D=2.5 m, v0=6.5 m/s, launch 70 degrees produces impact
  0.66588 m above the muzzle, descent 65.700 degrees, impact speed 5.4024 m/s,
  flight time 1.12454 s, apex 1.9015 m above the muzzle.
- Pollen: D=3 m, v0=7.5 m/s, launch 73 degrees produces impact
  0.63159 m above the muzzle, descent 70.664 degrees, impact speed 6.6225 m/s,
  flight time 1.36812 s, apex 2.6219 m above the muzzle.

Absolute apex height requires measured muzzle height. Neither trajectory
includes drag, spin, full ball envelope clearance through the cell opening,
nor hive dynamics.

For a fixed pivot-to-impact vector r and fixed impact speed, maximum angular
impulse occurs with momentum perpendicular to r. For impact coordinates
rx>0, rz>=0 and forward/downward flight, descent gamma=atan2(rx,rz).
That is an impact angle, not a launch angle. Vacuum launch angle depends
on range and height. For target height difference h and descent gamma:

    vx = sqrt(g D² / [2(h + D tan(gamma))])
    vz_launch = -vx tan(gamma) + g D / vx
    alpha_launch = atan2(vz_launch, vx)

If muzzle speed is limited, the best admissible trajectory also depends on
that cap and opening geometry. There is no universal optimal angle or
minimum ball count from the static tipping count alone. The supplied model
does not show that 2 nectar or 3 pollen are impossible.

## Official static calibration is more specific than a mass threshold

[FIRST Event Field Setup Guide §12](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide)
specifies tests with 8 pollen and 3 nectar + 3 pollen, at the back of the cell;
7 tossed pollen and 3 nectar + 2 tossed pollen must not tip in the specified
test. A pure 5-nectar test is not the published calibration requirement.
The team can use observed five-nectar behavior as additional test data.
These static tests do not uniquely determine inertia, energy barrier,
damper curve or the efficient dynamic shot angle.

## Next experiment

Calibrate both cells, fully compress the damper between reset tests, then
measure incoming/outgoing ball velocity and hive angle versus time. Sweep
impact position, speed, descent, spin and shot spacing; test both stable
states and realistic initial loads. Fit a loaded, dissipative time-domain
model and validate on held-out repeated shots. Dynamic model output should
be a reliability estimate with uncertainty, not an exact universal count.

## Verification artifacts

- `tools/fusion/mcp_review/mark4_selector_report.json`: CAD sweep and actual
  joint transforms, source preservation, saved/archive state.
- `tools/fusion/mcp_review/audit_hive_impulse.py`: recalculation and vacuum
  inverse-trajectory consistency assertions.
- `tools/fusion/mcp_review/hive_impulse_audit.json`: numeric results.
- The interactive canvas embeds the equations, assumptions and conditional
  vacuum trajectory search. It is not a measured shot optimizer.
