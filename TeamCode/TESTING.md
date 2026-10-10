# Desktop regression checks

From the repository root in PowerShell:

```powershell
./tools/run-desktop-tests.ps1
```

The script finds `JAVA_HOME`, the standard Android Studio bundled JDK, or a JDK
on PATH. You can also pass `-JdkHome 'C:/path/to/jdk'`. It compiles the pure Java
code for Java 8 and runs planner, tracker, state history, turret safety, Pedro
mapping/drive regressions, turret bench logic, audit regressions, and integrated
robot/scoring/autonomous scenarios. Any compilation error or failed assertion stops the script.
Output goes into the ignored `build/desktop-tests` directory.

These checks include Pedro core mapping and simulated drive behavior. They do
not exercise SDK hardware adapters, Android packaging, or physical hardware.
Build `:TeamCode:assembleDebug` separately in Android Studio
or with the Gradle wrapper. Validate the HuskyLens on the actual device: select
Color Recognition, enable Learn Multiple, teach yellow/red/blue, and configure
the corresponding camera IDs. Firmware 0.5.1 or later is needed to detect
multiple blocks of the same color. Measure thresholds, projection, and latency
under representative lighting before using detections to drive.

Mesh-validator regressions run in workspace temporary folders, including a
subprocess under `python -O` for every case:

```powershell
python -m unittest discover -s tools/tests -v
```

For the competition runtime and the measured configuration still required, see
[`doc/audits/BIOBUZZ_IMPLEMENTATION_2026-10-09.md`](../doc/audits/BIOBUZZ_IMPLEMENTATION_2026-10-09.md).
`RobotHardwareConfig.measured()` currently returns null because hardware is
undecided. Both competition OpModes display the reason and map no hardware.
Simulation constants and passing desktop tests do not establish bench readiness.

## Pickup plan lifecycle

The caller owns the committed plan. Each `plan` call returns a new immutable
snapshot using current target coordinates, confidence, robot pose, and model
costs. Hysteresis keeps the target order only while a competing route fails to
beat its newly estimated utility by the configured margin.

Pass `null` for the commitment after action completion, cancellation, failure,
or a localization reset. Inventory changes automatically invalidate a plan
created from a different onboard load. After confirmed intake, update the
onboard counts and remove the collected track before planning again. After
confirmed shooting, update the counts and clear the commitment even if a later
intake restores the same counts. Counts alone cannot identify an action event.

The Pickup Planner Test OpMode demonstrates collection with A, confirmed shot
completion with B, cancellation with X, and reset with Start. These are simulated
events; a real robot must use its inventory and mechanism sensors.
