# BIOBUZZ Simple Auto: setup and tuning guide

**OpMode:** `BIOBUZZ Simple Auto` (Autonomous, group Competition).
**Needs:** drive motors, `flywheel`, `hood` + `compression` servos, `intake` / `transfer` / `feeder` motors.
**Does not need:** Pedro, the turret, sensors, odometry, the Limelight. It runs on timers you measure.

> **Status:** the logic is tested on a laptop (about 100 checks: routes, shot sequencing, deadlines, dead
> encoder, late start). **It has never run on the robot.** Everything below that says "verify" is
> something only the robot can tell you. Do the checks in order, on blocks first.

The names it expects in the Robot Configuration are the same ones the rest of the repo uses:
`frontLeft`, `backLeft`, `frontRight`, `backRight`, `flywheel`, `hood`, `compression`, `intake`,
`transfer`, `feeder`. A missing device is listed on the Driver Station under HARDWARE; it doesn't crash.

## What it does

| Route | What happens |
|---|---|
| 0 LEAVE ONLY | Drive the PARK legs. Points for leaving, nothing else. Use this first, and as your fallback. |
| 1 PRELOAD + PARK | Spin the flywheel from the first loop. Optionally move to a shot spot. Fire the preload. Park. |
| 2 PRELOAD + PICKUP + SECOND SHOT + PARK | As 1, then drive out with the intake on, retrace back, fire again, park. |

Every route ends by parking. Safety behaviour you don't have to set up:

- Everything is commanded off 0.3 s before the 30 s mark.
- No new shot sequence starts after 24 s.
- At 25.5 s any remaining work is abandoned and the robot goes straight to parking.
- A shot never waits forever. If the flywheel never reaches speed it fires after 4 s and says so.
- Press **B** at any time: everything stops and latches.
- Blue is Red mirrored: every strafe distance flips sign. Tune Red, then just flip the alliance.

## Controls in INIT

| Input | Does |
|---|---|
| **X** / **B** | Blue / Red |
| **D-pad up/down** | Route 0, 1, 2 |
| **OPTIONS** (or Start on an Xbox pad) | Save the auto's numbers to the Control Hub |
| **LB held** + X / Y / A / B | Spin one wheel forward: X = front-left, Y = front-right, A = back-left, B = back-right |
| **RB held** | All wheels forward (robot should drive forward) |
| **Right trigger** | Strafe right |

The INIT screen shows the plan as a list of steps with their times, the best/worst-case total,
and any PROBLEM that would stop it from moving.

The wheel test is for the pit with the robot **on blocks**. Never use it in a match.

## Step by step

Tune the shooter first (see `doc/TUNING_QUICKSTART.md`). The auto reads hood, compression, power / rpm
and PIDF from the bench tuner's saved file, so whatever you save there is what the auto uses.

Open the Dashboard at `http://192.168.43.1:8080/dash` and the panel **Simple Auto**. Type a value and
press **Enter**. Press **OPTIONS** on the gamepad to save. **Dashboard edits alone are forgotten at the
next INIT**; OPTIONS is what makes them stick.

### 1. Wheel directions (robot on blocks)

1. INIT the auto. Hold **RB**. All four wheels must spin so the robot would go **forward**.
2. If one wheel spins backward, flip its `REVERSE_*` box in the Dashboard. Check again.
3. Hold the **right trigger**. The robot must strafe **right** (front-left and back-right forward, the other two back).
4. Press OPTIONS to save.

The defaults (left side reversed) are a guess for a typical mecanum build. Don't trust them.

### 2. Mechanism directions (on blocks, flywheel guard on)

The auto turns these on during a shot or a pickup. A wrong direction is the most likely first failure:

- `REVERSE_INTAKE`, `REVERSE_TRANSFER`, `REVERSE_FEEDER`: confirm each pushes balls the right way.
  A quick way: run route 2 with no balls and watch each motor on the feed pulse and the pickup.

### 3. Drive speed (route 0, on the floor, small space)

The auto turns inches into time using two speeds, `IN_PER_SEC_FORWARD` and `IN_PER_SEC_STRAFE`
(inches per second at `DRIVE_POWER`). The defaults are guesses.

1. Pick route 0. Set `PARK_FORWARD_IN` to 48 and `PARK_STRAFE_IN` to 0. Put the robot on the tiles with tape at the front wheel.
2. Run it. Measure how far it actually went.
3. In the Dashboard type that number in `MEASURED_FORWARD_IN`, tick `APPLY_MEASURED_SPEEDS`, press OPTIONS.
   `IN_PER_SEC_FORWARD` is now `distance / seconds actually driven`.
4. Run again. It should now land within about an inch or two. Repeat once if not.
5. Do the same for strafe: `PARK_FORWARD_IN` = 0, `PARK_STRAFE_IN` = 48, `MEASURED_STRAFE_IN`.
6. A battery at a different voltage changes these. Use a battery you'll actually compete on, and re-check.

Dead reckoning drifts. Slip, a bumped wall or a different battery moves the robot a few inches, and it
never knows. Aim to reach a spot you can **square against a wall or tile edge** and keep legs short.

### 4. The shot

1. Choose where to shoot from. Put the robot there by hand and fire a few balls from the bench tuner / TeleOp to find a working hood and rpm. Save it in the bench tuner.
2. Measure how far the robot has to drive from its start to that spot. Put it in `SHOT_MOVE_FORWARD_IN` / `SHOT_MOVE_STRAFE_IN`. (0 and 0 means shoot from the start.)
3. Set `FEED_PULSE_SEC` so **one pulse fires one ball**. Too short and nothing leaves; too long and two go. Start at 0.35 and adjust by 0.05.
4. Run route 1 with 3 balls. The screen shows, for each ball, the rpm it was fed at. The dip after each ball and the recovery are what you're watching.
5. If the first shot is low, raise `MIN_SPINUP_SEC` so it waits longer before the first ball.
6. If the flywheel cannot reach speed within `SPINUP_MAX_SEC`, you'll see `SPIN-UP TIMEOUT`. Raise it, or lower the shot power.

**Open loop vs velocity** (`OPEN_LOOP` box):

- *Open loop* (default): fixed power, like your OnBot tests at about 90%. The auto learns the flywheel's steady rpm and feeds the next ball once the speed has come back within 3% of it. The first shot can leave a few percent under the final speed (it can only call the speed "steady" once it stops climbing).
- *Velocity*: untick `OPEN_LOOP` and use the PIDF you tuned on the bench. Ready is exact (within 3% of `SHOT_RPM`).
  Prefer this once your PIDF is good.

If the encoder isn't working you'll see `NO ENCODER SIGNAL: using timer`. The auto then waits
`TIMED_SPINUP_SEC` and keeps going. Fix the encoder when you can.

### 5. Pickup (route 2)

- `PICKUP_FORWARD_IN` / `PICKUP_STRAFE_IN` is how far to drive out with the intake on, and it retraces the same path back. Measure from the shot spot to the balls.
- `PICKUP_SETTLE_SEC` is a short pause at the far end for the last ball to get in.
- Do route 2 only after route 1 works five times in a row.

### 6. Park

`PARK_FORWARD_IN` / `PARK_STRAFE_IN` is where the robot finishes. This leg runs even if time is short. Measure it from the shot spot.

## What the screen is telling you

| Line | Meaning |
|---|---|
| Time | Seconds used of 30. |
| Step / status | The current step, and what it is doing (spinning up, feeding shot 2 of 3, recovering). |
| EVENT | The last notable thing: a timeout, a skipped shot, "RUNNING OUT OF TIME". |
| Ready | Whether the flywheel is at speed, the learned baseline rpm, and NO ENCODER SIGNAL if it never moved. |
| Shot N | The rpm each ball was fed at. Use this to tune. |
| Calibration | Full-power seconds driven forward/strafe, for the speed calibration above. |

## Order to bring it up

1. Wheel directions (step 1).
2. Route 0 with a measured speed (step 3). **This alone is worth points. Do it first.**
3. Mechanism directions (step 2).
4. Route 1 with the shot from the start position (shot move 0, 0). Then add a shot move if you need one.
5. Route 2.
6. Save, then run each route with the Red and the Blue alliance.

## Known limits

- **No heading correction.** Drive legs are timed with no IMU or odometry, so the robot can crab a few degrees on a long leg. Keep legs short, or add the IMU hold later.
- **No ball sensing.** It feeds a fixed number of pulses. If a ball jams or the magazine is short, it doesn't know.
- **Turret stays where it is.** Point it where you want before START (centre or fixed angle). The auto does not move it.
- **One compression/hood setting.** It uses the pollen setting from the bench file for every ball.
- It can't see the other robots or the field. If an alliance partner is in your path, it will push into them. Use `START_DELAY_SEC` and route choice to coordinate.
- Once Pedro is tuned (see `doc/BIOBUZZ_REVIEW_AND_AUTO_PLAN.md`), the fixed-route auto and Pedro paths are the way to
  get precise movement. This auto is the one you can have working this week.
