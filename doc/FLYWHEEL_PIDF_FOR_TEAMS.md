# Flywheel PIDF auto-tuner for FTC teams

Run an empty flywheel from FTC Dashboard to measure feedforward and tune the software velocity loop.
The tuner checks speed at three open-loop power levels, compares four P gains on a speed step, and
verifies steady speed with I enabled. The resulting D is zero. The flywheel stops at the end of a run.

## Add it to your project

Use an FTC Android Studio project with FTC Dashboard. This tuner was built with FTC SDK 12.0 and
Dashboard 0.6.0. If your project doesn't already include Dashboard, add the following to the existing
`repositories` and `dependencies` blocks in `build.dependencies.gradle`, respectively.

```groovy
// repositories
maven { url 'https://maven.brott.dev/' }

// dependencies
implementation 'com.acmerobotics.dashboard:dashboard:0.6.0'
```

Copy the ZIP's `TeamCode/src/main/java` tree into the same location in your project, keeping the Java
package folders. If you're working from GitHub, copy these five files to their matching paths under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

- [`test/TurretShooterBenchOpMode.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TurretShooterBenchOpMode.java) runs the bench.
- [`test/DashboardTunables.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/DashboardTunables.java) connects the bench to Dashboard.
- [`scoring/FlywheelAutoTuner.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/scoring/FlywheelAutoTuner.java) measures and validates the gains.
- [`scoring/ShooterCalibrationStorage.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/scoring/ShooterCalibrationStorage.java) saves the calibration.
- [`robot/FlywheelPair.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FlywheelPair.java) runs the battery-compensated velocity loop.

From this repository, run `python tools/export-flywheel-tuner.py` to generate
`build/FTC-Flywheel-PIDF-Tuner.zip` for sharing.

## Match your hardware

1. In `TurretShooterBenchOpMode.java`, set `USE_BENCH_SERVOS = false` for flywheel-only tuning.
   The optional hood and compression code is set up for BioBuzz's Axon servos and linkage.
2. Set `PRIMARY_FLYWHEEL_NAME` and `FLYWHEEL_DIRECTION` for your motor. Use a configured REV Hub
   motor port with the encoder connected. Set `SECOND_FLYWHEEL_NAME` to `""` for one motor. For
   two motors driving the same flywheel, set it to the second motor's configured name, connect both
   encoders, and set `FlywheelPair.SECOND_OPPOSITE` if the motors need opposite directions.
   Check that the motors drive the wheel together and that Dashboard shows positive RPM in open loop.
3. Set `MAX_RPM_LIMIT` for your mechanism. The tuner sweeps at 45%, 65%, and 85% power by default.
   If those levels are outside your mechanism's operating range, set three safe, increasing powers
   in `FlywheelAutoTuner.SWEEP_POWERS` before running it.
4. In Dashboard, set `TICKS_PER_REV` to the primary encoder's ticks per **flywheel** revolution.
   With a direct drive and a 28-tick encoder, use 28. If that motor turns twice per flywheel
   revolution, use 56. Set `TARGET_RPM` to the speed you intend to shoot at.

## Tune and verify

1. Run **Bench → Shooter & Hood Bench Tuner** and open **FTC Dashboard → Configuration →
   Shooter Bench**. Keep the shooter empty, keep clear of the wheel, and let it come to rest.
2. Check `AUTO_TUNE_PIDF` and submit that row. Each power level has up to 15 seconds to settle.
   Check `AUTO_TUNE_PIDF` again to cancel a running tune, or press **STOP** on the Driver Hub
   to end the OpMode. If it fails, use the Dashboard message to diagnose the run before retrying.
3. After **Done**, the tuner loads the gains and selects `VELOCITY_MODE`. Turn on `FLYWHEEL_ON`,
   graph `targetRpm` and `actualRpm`, and wait for speed to settle. Feed balls to verify recovery
   at your usual shot cadence. Dashboard reports **Last ball dip** and recovery time.
4. Turn off `FLYWHEEL_ON`, then check and submit **`SAVE_CALIBRATION`** on its own row.
   The tuner displays **Calibration saved to Control Hub**.

## Use the gains in your robot code

The gains are for the battery-compensated **software** loop in `FlywheelPair`. Keep
`FlywheelPair.SOFTWARE_VELOCITY` enabled and use the same motor names and directions as the bench.
`SAVE_CALIBRATION` stores the gains and the encoder scale. Your match OpMode can load them explicitly.

```java
// During initialization, after mapping FlywheelPair and setting its direction.
ShooterCalibrationStorage.Data tune = ShooterCalibrationStorage.load();
flywheel.setPower(0);
flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
        new PIDFCoefficients(tune.p, tune.i, tune.d, tune.f));
flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Every loop while enabled, where requestedRpm is your shot speed.
flywheel.setVelocity(requestedRpm * tune.ticksPerRev / 60.0);

// When disabled or when stopping the OpMode.
flywheel.setPower(0);
```

Use only one controller to command the flywheel from an OpMode. Verify speed and shot recovery with
your own hardware and battery. Recheck the tune after changes to the motor, gearing, or flywheel.

See the [repository license](../LICENSE).
