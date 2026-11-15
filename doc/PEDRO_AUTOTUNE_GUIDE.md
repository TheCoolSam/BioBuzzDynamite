# Pedro Pathing AutoTune guide

Pedro Pathing 3 tunes itself. A web page hosted on the robot drives the robot through short tests,
measures it, and prints Java for you to paste. You do not tune gains by hand.

> **Status:** wired into this repo, compiles, **never run on the robot.** The steps below follow the
> official Pedro docs ([Tuning](https://pedropathing.com/docs/pathing/tuning),
> [Foresight](https://pedropathing.com/docs/pathing/tuning/foresight),
> [Test](https://pedropathing.com/docs/pathing/tuning/test)). Anything the robot does that differs from
> this guide is a bug in the guide; trust the robot and the Pedro docs.

## What was missing before

Your project already had Pedro and the AutoTune library, but not the AutoTune *procedures* (the tuners).
Those are source files in Pedro's Quickstart repo that you copy into your project. They are now in
`TeamCode/.../pedro/procedures/`, listed in [Tuning.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Tuning.java).
Pedro was also bumped to the versions the quickstart uses (`revhub 3.0.2`, `tuning 1.0.3`); 3.0.2 fixes
controller derivative and integral bugs.

Where results go: [PedroTuned.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/PedroTuned.java).
Every field in it is a placeholder today.

## Before you start

1. The robot must be on the floor with room to drive about 6 feet forward and 4 feet left. Some
   procedures drive the robot quickly. Keep people clear and a hand on the Stop button.
2. A **charged battery** you will actually use. Velocities depend on battery voltage.
3. Your laptop on the Robot Controller's Wi-Fi (the same network used for the Dashboard).
4. Localizer: we use a **goBILDA Pinpoint with two perpendicular dead wheels**, so
   `PedroConstants.LOCALIZER` stays `PINPOINT`. The Pinpoint Tuner is the one to run. In the Robot
   Configuration the Pinpoint must be an I2C device on the Control/Expansion Hub, type
   **goBILDA Pinpoint Odometry Computer**, named exactly `pinpoint` (`PedroConstants.PINPOINT_NAME`).
   The Pinpoint has its own IMU, so you do not need the Hub IMU for Pedro.
   Have these ready for the Pinpoint Tuner: the pod type (goBILDA swing-arm, goBILDA 4-bar, or custom
   pods with their own wheel diameter and ticks per revolution), and a tape measure.
5. Build and install the app (Android Studio, Run). The tuner procedures show up after install.

Open **http://192.168.43.1:10158** in a browser. Pick a procedure from the list.

## The order

Do these in order. Each one needs the one before it pasted in.

### 1. Mecanum Tuner: wheel names and directions

1. Pick **Mecanum Tuner**. Enter your four drive motor names exactly as in the Robot Configuration
   (this repo assumes `frontLeft`, `backLeft`, `frontRight`, `backRight`).
2. Each wheel spins one at a time. After each, tell it whether the wheel spun **forward** or reversed.
   The page shows a diagram of which wheel is spinning and which way forward is.
3. When it finishes, open the **Java** tab. Copy the `drivetrainConfig` block.
4. In `PedroTuned.java`, **replace** the `drivetrainConfig` statement with it. Keep the field name.
   Add any missing imports (Android Studio offers them: `DcMotorSimple`, `MecanumConfig`).

This also gives you the real answer for the Simple Auto's wheel directions.

### 2. Localizer tuner: where the robot thinks it is

Pick the one for your hardware:

| You have | Procedure | Paste over this field in `PedroTuned.java` |
|---|---|---|
| goBILDA Pinpoint (**this is us**) | Pinpoint Tuner | `pinpointConfig` |
| Two dead wheels + IMU | Two Wheel Tuner | `twoWheelConfig` |
| Three dead wheels | Three Wheel Tuner | `threeWheelConfig` |
| Three dead wheels + IMU | Three Wheel IMU Tuner | `threeWheelImuConfig` |

The generated block is named `localizerConfig`. **Rename it to the field name in the table** when you paste,
so the factory finds it. Follow the on-screen steps (it asks you to push the robot a measured distance,
check each pod's direction, and find the pod offsets). Measure carefully with a tape measure:
a bad offset makes every later step worse.

Then use **Tests -> Odometry Test** (step 4) as a quick check before moving on.

If you have no dead wheels at all: Pedro needs a localizer. Drive-motor encoders alone are not supported
here. In that case use the [Simple Auto](SIMPLE_AUTO_GUIDE.md) and consider a Pinpoint.

### 3. Foresight Tuner: speed, braking and path-following gains

This is the big one. It runs the robot through forward and strafe velocity, deceleration, heading,
and braking tests, and then tunes the translational and heading gains.

1. Pick **Foresight Tuner**. Read each step's description on the page. Some steps run the robot hard
   in a straight line, then brake. Others spin it in place. Give it space.
2. When finished, open the **Java** tab and copy the `foresightConfig` block.
3. In `PedroTuned.java`, replace the `foresightConfig` statement with it.

### 4. Tests: prove it before trusting it

Pick **Tests**. In this order:

1. **Odometry Test**: drives forward, left and counterclockwise and confirms the localizer agrees.
2. **Line Test**: drives 48 inches forward and back, repeatedly. It should land where it started.
3. **Curved Test** and **Interpolation Test**: curved paths, various heading behaviours.
4. **Hold Test**: holds position. Push the robot and watch it push back.

If the Line Test is wrong by more than a couple of inches, redo step 2 (offsets and scales) before
touching anything else.

### 5. Confirm it

Only after the Tests pass: in `PedroConstants.java` set `HARDWARE_AND_TUNING_CONFIRMED = true`.
That flag is the repo's gate; `PedroFactory.create` refuses to build a Follower without it.

Also check [PedroConstants.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/PedroConstants.java):
the hub orientation is a placeholder (logo up, USB forward). If your localizer uses the Control Hub IMU
(two-wheel or three-wheel with IMU), it must match how the hub is actually mounted.

## What this unlocks, and what it doesn't

- **Pedro paths for autonomous:** yes. With a tuned follower you can build real paths with
  `PedroFactory.create(hardwareMap)`.
- **`BIOBUZZ Fixed Route` and `BIOBUZZ TeleOp`:** not by themselves. They also need the turret
  commissioning and the measured shooter/vision values (see `doc/BIOBUZZ_REVIEW_AND_AUTO_PLAN.md`).
- **`BIOBUZZ Simple Auto`:** works without any of this, and can take over driving once Pedro is tuned
  (a Pedro-based version is the natural next step).

## If something goes wrong

- *The web page won't load:* the app must be running on the Robot Controller and your laptop on its
  Wi-Fi. Try `http://192.168.43.1:10158` (not `https`). Rebuild and reinstall if no procedures are listed.
- *A procedure fails at the start:* the usual cause is a wrong hardware name. Compare the names you
  typed with the Robot Configuration character for character.
- *The robot drives the wrong way in a test:* stop, redo step 1 (directions) and check the localizer
  step's pod directions.
- *Results look odd after a battery swap:* tune with the battery you will compete on.
- Pedro's own docs and Discord are the authority on AutoTune behaviour; this repo only wires it in.
