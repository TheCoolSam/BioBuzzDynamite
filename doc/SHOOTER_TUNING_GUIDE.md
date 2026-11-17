# Shooter tuning guide: flywheel PIDF, compression, hood

For people who have never tuned a shooter. Do the parts **in order**. Each part says what you are
doing, why, and how you know it worked. Use the OpMode **Shooter & Hood Bench Tuner** (group Bench).
Setup and the Dashboard check are in [TUNING_QUICKSTART.md](TUNING_QUICKSTART.md) section 0.

## Safety (read first, every session)

- Eye protection on everyone near the robot. Keep people and cables out of the ball exit path.
- **Hold B / Circle = emergency stop.** It cuts the flywheel and relaxes both servos. One person's
  only job is to have a thumb on it.
- Never force a servo into a mechanical stop. If a servo buzzes, stalls or gets hot, back off right away.
- Use a **charged battery**. Note the `Battery V` line. Retest with a half-used battery at the end,
  because flywheel speed changes with voltage.
- Pressing START moves both servos to the saved positions straight away. Clear the mechanism first.

## Controller map (PS5)

| Button | What it does |
|---|---|
| Cross (X) | Flywheel on / off |
| Back / Share | Switch OPEN LOOP and VELOCITY mode (flywheel stops when you switch) |
| D-pad up / down | OPEN LOOP: power +/- 5%. VELOCITY: `TARGET_RPM` +/- 50 |
| D-pad left / right | Compression -/+ 0.01 for the active ball |
| Hold L2 + D-pad | up/down = P, left/right = D |
| Hold R2 + D-pad | up/down = F, left/right = I |
| Right stick (up/down) | Hood position, continuous |
| LB / RB | Hood -0.01 / +0.01 |
| Left stick (up/down) | Compression, continuous |
| Triangle | Switch active ball (POLLEN / NECTAR) |
| Square | Servos on / relaxed (unpowered, you can move them by hand) |
| Right stick click | Apply the suggested F and P (open loop only, after STEADY shows) |
| Options | Save to the Control Hub |
| B / Circle (hold) | Emergency stop |

Anything can also be typed into the Dashboard panel at `http://192.168.43.1:8080/dash`
(type the value and press **Enter**). Graph tab: tick `targetRpm` and `actualRpm`.

---

## Part 1: Flywheel PIDF

**What PIDF is.** In velocity mode you ask for a speed (rpm) and the motor controller adjusts power to
hold it. Four numbers control how:

- **F (feedforward):** the power the motor needs just to *hold* a speed. This does most of the work.
- **P (proportional):** extra push proportional to how far the speed is off. Fixes the remaining error fast.
- **I (integral):** slowly adds power if the speed stays a little off. Usually left at 0.
- **D (derivative):** damps overshoot. Usually left at 0.

The tuner measures F from your real motor, so you do not guess it.

### 1A. Measure F in open loop

1. INIT the tuner, press START. Check the top of the screen says `MODE: OPEN LOOP`.
2. Press **Cross**. The wheel goes to `OPEN_LOOP_POWER` (90%). Watch `actualRpm`.
3. Wait until `STEADY` appears (the speed has held still for 1.5 s). Write down:
   `rpm`, `power`, `volts`, `Suggested F`. For a 6000 rpm goBILDA at 90% expect roughly 5000 to 5500 rpm
   and F near 11 to 12. Very different numbers mean something is off (see Troubleshooting).
4. **Click the right stick.** That loads F, sets P to 10% of F, and sets I and D to 0.
5. Press **Cross** to stop the wheel.

### 1B. Tune P in velocity mode

1. Press **Back/Share** (velocity mode). Set `TARGET_RPM` to about 3000 (D-pad up/down). Pick a target
   **well below the max** you measured (under about 80%) so the controller has headroom.
2. Press **Cross**. Watch the graph. A good result: actual climbs to the target, then sits on it.
3. Read the graph and fix the first problem you see:

| What you see | Meaning | Change |
|---|---|---|
| Settles clearly **below** target | F too low | Raise F by 0.25 (R2 + D-pad up) |
| Settles clearly **above** target | F too high | Lower F by 0.25 |
| Reaches target but slowly | P too low | Raise P by 0.25 (L2 + D-pad up) |
| Wobbles up and down around target | P too high | Lower P by 0.25 |
| Overshoots once, then settles | A little more damping | Lower P a bit, or add D 0.05 |
| Small constant error left over | Last resort | I = 0.01 to 0.05 (anything bigger can wind up and oscillate) |

4. Change **one number at a time**, wait for the graph to settle, then judge.
5. Repeat at other targets (for example 2500, 3500, 4500). The same PIDF should work at all of them.
   If one target is bad and the others are fine, that target is too close to max speed.
6. Check `PIDF on hub ... [SYNCED]`. `MISMATCH` means the write failed.

**Done when:** at three different targets, `actualRpm` reaches the target in about a second or less,
stays within about 1 to 2% of it, and does not ring.

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

1. Press **Options** (or tick `SAVE_CALIBRATION`). The screen says `CALIBRATION SAVED TO DISK`.
2. Write down final `P I D F` and give them to the programmers. They go in
   `RobotHardwareConfig.flywheelPidf` (about line 83) so the real match code uses them.
   The bench file on the hub is only used by the bench tuner and the Simple Auto.

---

## Part 2: Compression

**What it is.** The `compression` servo changes how hard the ball is squeezed between the flywheel and
the hood. More squeeze = more grip and more spin on the ball. Too little = the ball slips and shots are
weak and inconsistent. Too much = the wheel bogs down, the ball can jam, and the servo is strained.
`POLLEN` and `NECTAR` each get their own number because the balls are different sizes.
Triangle switches which one you are editing (the screen shows `Active ball`).

### 2A. Find the safe range (no flywheel)

1. Flywheel **off**. Press START so the servos are powered.
2. Use the left stick or D-pad left/right to move compression in **0.01 steps**, slowly.
3. Put a ball in the shooter by hand. Write down:
   - `NO TOUCH`: the value where the ball is free.
   - `FIRST CONTACT`: the value where the ball just touches the wheel.
   - `TOO FAR`: the value where it feels tight or the servo strains. Never go there with the wheel on.
4. Stay at least 0.03 away from each mechanical end.
5. Repeat with the other ball type (Triangle).

### 2B. Pick the working value with shots

Use velocity mode with your tuned PIDF, one fixed `TARGET_RPM`, and one fixed hood position.

1. Start about 0.02 past `FIRST CONTACT`.
2. Fire 5 balls. For each: where it landed (same target every time), and the `Last ball dip`.
3. Raise compression by 0.02 and repeat. Keep going until shots get worse or the dip gets large.
4. Write the results in the table below. Choose the setting with the **most consistent landing spot**
   and a small dip, not the one that goes farthest once.
5. Do pollen and nectar separately. Save with **Options**.

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

1. Flywheel off. Move the hood slowly with the right stick, or LB/RB in 0.01 steps.
2. Write down the lowest and highest values that move freely with no binding or buzzing. Keep 0.03 away
   from each end.
3. At 4 or 5 evenly spaced values, hold a phone with a level app against the hood and write down the angle.
   You now know what each number means (`HOOD_POSITION` to angle).

### 3B. Shot tests at one distance

1. Put the robot at a fixed distance (tape on the floor) from a target. Fix `TARGET_RPM` and compression.
2. Fire 5 balls at 4 different hood values. Record where they land (short, good, long, bounced back out).
3. Pick the value with the most hits and the fewest bounce-backs. Save with **Options**.
4. If shots land short even at the steepest hood, raise rpm. If they come back out of the opening,
   try more hood angle first, then slightly less compression (less spin).

### 3C. Build a table for other distances

Repeat 3B at 3 or 4 distances (for example 1, 1.5, 2, 2.5 m). Fill in the table below. This is the
data the programmers need to turn "distance to the goal" into a hood and rpm setting.

---

## Recommended order for the whole session

1. Safety check, battery, servos on and moving freely.
2. Part 2A and 3A (servo ranges), flywheel off.
3. Part 1A and 1B (F, then P).
4. Part 2B and 3B shot tests at one distance.
5. Part 1C (ball dip) once compression and hood are chosen.
6. Part 3C (distance table). Save with Options. Give the numbers to the programmers.
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

- **`actualRpm` stays 0 with the wheel powered:** `WARNING: 0 encoder ticks/s`. Check the 4-pin encoder cable.
- **rpm looks wrong by a fixed factor:** `TICKS_PER_REV` must be 28 for the 6000 rpm goBILDA. The
  `Flywheel motor type` line shows the configuration's number, which may be different and can be ignored
  as long as `TICKS_PER_REV` is 28.
- **`STEADY` never shows:** open loop, flywheel on, and the speed has to stay within about 1.5% for 1.5 s.
  A loose belt or a sagging battery makes it drift.
- **Wheel hunts even at tiny P:** P is probably bigger than F. P should start near 10% of F.
- **Dashboard edit did nothing:** you did not press Enter, or the laptop is not on the robot Wi-Fi.
  Check `Dashboard edits received` on the Driver Station.
- **Servo does not move:** servos are relaxed. Press Square so it says `Servos: ENGAGED`.
- **Different results every day:** battery voltage, ball wear, and wheel wear all change the shot.
  Log the volts every session.
