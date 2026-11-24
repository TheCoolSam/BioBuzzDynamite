# Shooter tuning guide: flywheel PIDF, compression, hood

For people who have never tuned a shooter. Do the parts **in order**. Each part says what you are
doing, why, and how you know it worked. Use the OpMode **Shooter & Hood Bench Tuner** (group Bench).
Setup and the Dashboard check are in [TUNING_QUICKSTART.md](TUNING_QUICKSTART.md) section 0.

## Safety (read first, every session)

- Eye protection on everyone near the robot. Keep people and cables out of the ball exit path.
- Use the Driver Hub **STOP** button to end the OpMode immediately. In Dashboard, `STOP_ALL`
  stops the flywheel and releases both servos while keeping the OpMode open.
- Back off if a servo buzzes, stalls or gets hot. Keep the linkage clear before enabling the servos.
- Start with a charged battery, then repeat the speed and recovery checks with a partly used one.
- At START the servos first receive their saved starting commands. If the linkages moved while
  unpowered, they can move back to those positions when enabled.

## Controls

- **D-pad up/down** moves the hood. **D-pad left/right** moves compression for the selected ball.
  Hold for continuous motion or tap for a small change.
- In **FTC Dashboard > Configuration > Shooter Bench**, select the ball with `NECTAR_BALL` (off
  selects pollen). Enter exact hood and compression targets there. `SERVOS_ENABLED` controls PWM.
- Use Dashboard for all flywheel controls, PIDF, and `SAVE_CALIBRATION`. Press Enter or the check
  mark to submit config edits. Graph `targetRpm` and `actualRpm` in Dashboard.

---

## Part 1: Flywheel PIDF

**What PIDF is.** In velocity mode you ask for a speed (rpm) and the motor controller adjusts power to
hold it. Four numbers control how:

- **F (feedforward):** the power the motor needs just to *hold* a speed. This does most of the work.
- **P (proportional):** extra push proportional to how far the speed is off. Fixes the remaining error fast.
- **I (integral):** slowly adds power if the speed stays a little off. Auto-tune checks this gain during the speed hold.
- **D (derivative):** damps overshoot. Usually left at 0.

### 1A. Auto-tune

1. INIT and START **Shooter & Hood Bench Tuner**. In Dashboard, set `TARGET_RPM` to your shot
   speed and verify `TICKS_PER_REV`. Leave `FLYWHEEL_ON` off and let the wheel stop.
2. Clear the shooter and set `AUTO_TUNE_PIDF`. The tuner measures open-loop feedforward, compares
   P values on a speed step, and verifies the chosen P, I, and F on a steady hold. D stays at zero.
   Each open-loop power step has up to 15 seconds to settle. Dashboard shows how long the current
   power step has run.
3. Wait for **Done** in Dashboard. The motor stops and the new gains are loaded in velocity mode.
   Setting `AUTO_TUNE_PIDF` again while it is running cancels it. If a run fails or is cancelled,
   the previous gains stay in the Dashboard fields.
4. Set `FLYWHEEL_ON` to test the result. Watch how quickly `actualRpm` reaches `targetRpm` and how
   steadily it holds. Then test shot recovery in Part 1C.

### 1B. Manual tuning in Dashboard

1. Clear `VELOCITY_MODE`, set `OPEN_LOOP_POWER`, and turn on `FLYWHEEL_ON`. Wait for `STEADY`.
2. Set `APPLY_SUGGESTED_PIDF` to load the measured F with P and I at 1.2 times F and D at zero.
3. Turn off `FLYWHEEL_ON`, select `VELOCITY_MODE`, and set `TARGET_RPM`. Pick a speed with headroom
   below the estimated maximum. Turn on `FLYWHEEL_ON` and graph the response.
4. Edit one of `P`, `I`, `D`, or `F` at a time. P controls the initial correction, I removes
   steady speed error, and F supplies most of the power at the target. Compare results at the
   speeds you shoot at.

### 1C. Test with real balls (the part that matters)

Firing a ball steals speed from the wheel. The next ball goes short if the wheel has not recovered.

1. With the wheel settled at the target, feed one ball. The `Last ball dip` line shows how many rpm it
   cost and how long recovery took.
2. Fire at least 5 balls and note the dip and recovery each time.
3. Try small P changes (up 0.25 at a time). Better = smaller dip and shorter recovery with no wobble.
   Stop raising P when the idle wheel starts to ring.
4. If the wheel never recovers (`NEVER`), your target is too close to max speed for the battery.
   Lower the target or use a fresher battery.

### 1D. Save it

1. Set `SAVE_CALIBRATION` and look for **Calibration saved to Control Hub**. The bench tuner and
   BioBuzz Simple Auto load the saved calibration.
2. Record the final P, I, D, and F for the competition configuration in
   `RobotHardwareConfig.flywheelPidf`.

---

## Part 2: Compression

**What it is.** The `compression` servo changes how hard the ball is squeezed between the flywheel and
the hood. More squeeze = more grip and more spin on the ball. Too little = the ball slips and shots are
weak and inconsistent. Too much = the wheel bogs down, the ball can jam, and the servo is strained.
`POLLEN` and `NECTAR` each get their own number because the balls are different sizes.
Select which one to edit with `NECTAR_BALL` in Dashboard.

### 2A. Find the safe range (no flywheel)

1. Flywheel **off**. Press START so the servos are powered.
2. Tap or hold D-pad left/right to move compression slowly.
3. Put a ball in the shooter by hand. Write down:
   - `NO TOUCH`: the value where the ball is free.
   - `FIRST CONTACT`: the value where the ball just touches the wheel.
   - `TOO FAR`: the value where it feels tight or the servo strains. Never go there with the wheel on.
4. Stay at least 0.03 away from each mechanical end.
5. Select the other ball type in Dashboard and repeat.

### 2B. Pick the working value with shots

Use velocity mode with your tuned PIDF, one fixed `TARGET_RPM`, and one fixed hood position.

1. Start about 0.02 past `FIRST CONTACT`.
2. Fire 5 balls. For each: where it landed (same target every time), and the `Last ball dip`.
3. Raise compression by 0.02 and repeat. Keep going until shots get worse or the dip gets large.
4. Write the results in the table below. Choose the setting with the **most consistent landing spot**
   and a small dip, not the one that goes farthest once.
5. Do pollen and nectar separately. Save with `SAVE_CALIBRATION`.

### 2C. After you change compression

Compression changes the load on the wheel, so go back and re-check `Last ball dip` and the wheel
recovery from Part 1C.

---

## Part 3: Hood angle

**What it is.** The `hood` servo tilts the exit path. A **flatter** hood sends the ball on a lower, faster
path. A **steeper** hood sends it higher with a steeper drop. The ball should drop into the goal opening
instead of skimming across it (see `BIOBUZZ_REVIEW_AND_AUTO_PLAN.md`, "What backspin does").
The hood value is a servo position from 0 to 1, **not degrees**.

### 3A. Find safe travel and read the real angle

1. Flywheel off. Tap or hold D-pad up/down to move the hood slowly.
2. Write down the lowest and highest values that move freely with no binding or buzzing. Keep 0.03 away
   from each end.
3. At 4 or 5 evenly spaced values, hold a phone with a level app against the hood and write down the angle.
   You now know what each number means (`HOOD_POSITION` to angle).

### 3B. Shot tests at one distance

1. Put the robot at a fixed distance (tape on the floor) from a target. Fix `TARGET_RPM` and compression.
2. Fire 5 balls at 4 different hood values. Record where they land (short, good, long, bounced back out).
3. Pick the value with the most hits and the fewest bounce-backs. Save with `SAVE_CALIBRATION`.
4. If shots land short even at the steepest hood, raise rpm. If they come back out of the opening,
   try more hood angle first, then slightly less compression (less spin).

### 3C. Build a table for other distances

Repeat 3B at 3 or 4 distances (for example 1, 1.5, 2, 2.5 m). Fill in the table below. This is the
data the programmers need to turn "distance to the goal" into a hood and rpm setting.

---

## Recommended order for the whole session

1. Safety check, battery, servos on and moving freely.
2. Part 2A and 3A (servo ranges), flywheel off.
3. Part 1A for auto-tune, or Part 1B for manual PIDF tuning.
4. Part 2B and 3B shot tests at one distance.
5. Part 1C (ball dip) once compression and hood are chosen.
6. Part 3C (distance table). Save with `SAVE_CALIBRATION` and record the results.
7. Last: half-used battery check, and the Limelight Auto Aim check
   (see TUNING_QUICKSTART.md section 2).

## Data tables (copy these and fill them in)

**Servo ranges**

| Item | No touch | First contact | Too far | Min free | Max free |
|---|---|---|---|---|---|
| Pollen compression | | | | n/a | n/a |
| Nectar compression | | | | n/a | n/a |
| Hood | n/a | n/a | n/a | | |

**Hood angle read with a phone**

| `HOOD_POSITION` | Angle (deg) |
|---|---|
| | |

**Flywheel**

| Open-loop power | Steady rpm | Volts | Suggested F |
|---|---|---|---|
| | | | |

| Target rpm | P | I | D | F | Reaches target in (s) | Ring? |
|---|---|---|---|---|---|---|
| | | | | | | |

**Shot log (one row per ball)**

| Distance | Ball | rpm | Hood | Compression | Ball dip (rpm) | Recovery (s) | Result (hit / short / long / bounced) |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

## Troubleshooting

- **`actualRpm` stays 0 with the wheel powered:** check the 4-pin encoder cable.
- **rpm looks wrong by a fixed factor:** `TICKS_PER_REV` must be 28 for the 6000 rpm goBILDA. The
  `Flywheel motor type` line shows the configuration's number, which may be different and can be ignored
  as long as `TICKS_PER_REV` is 28.
- **`STEADY` never shows:** open loop, flywheel on, and the speed has to stay within about 1.5% for 1.5 s.
  A loose belt or a sagging battery makes it drift.
- **Wheel hunts in velocity mode:** review the actual-RPM graph and compare lower P or I values.
- **Dashboard edit did nothing:** you did not press Enter, or the laptop is not on the robot Wi-Fi.
  Check `Dashboard edits received` on the Driver Station.
- **Servo does not move:** check `SERVOS_ENABLED` in Dashboard and the hardware status in INIT.
- **Different results every day:** battery voltage, ball wear, and wheel wear all change the shot.
  Log the volts every session.
