[CmdletBinding()]
param([string]$JdkHome, [string]$PedroCoreJar)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$sourceRoot = Join-Path $repoRoot 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
$classes = Join-Path $repoRoot 'build/desktop-tests/classes'

if (-not $JdkHome) { $JdkHome = $env:JAVA_HOME }
if (-not $JdkHome) {
    $androidJdk = Join-Path ${env:ProgramFiles} 'Android/Android Studio/jbr'
    if (Test-Path -LiteralPath $androidJdk) { $JdkHome = $androidJdk }
}
if ($JdkHome) {
    $javac = Join-Path $JdkHome 'bin/javac.exe'
    $java = Join-Path $JdkHome 'bin/java.exe'
} else {
    $javac = (Get-Command javac -ErrorAction Stop).Source
    $java = (Get-Command java -ErrorAction Stop).Source
}
if (-not (Test-Path -LiteralPath $javac) -or -not (Test-Path -LiteralPath $java)) {
    throw 'A JDK is required. Set JAVA_HOME or pass -JdkHome with its installation directory.'
}

New-Item -ItemType Directory -Path $classes -Force | Out-Null
if (-not $PedroCoreJar) {
    $pedroCache = Join-Path $env:USERPROFILE '.gradle/caches/modules-2/files-2.1/com.pedropathing/core'
    $PedroCoreJar = Get-ChildItem -LiteralPath $pedroCache -Recurse -Filter 'core-*.jar' |
        Where-Object { $_.Name -notmatch 'sources|javadoc' } |
        Sort-Object { [version]($_.Name -replace '^core-|\.jar$', '' -replace '[^0-9.].*$', '') } -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $PedroCoreJar -or -not (Test-Path -LiteralPath $PedroCoreJar)) {
    throw 'Pedro core is required. Resolve Gradle dependencies or pass -PedroCoreJar.'
}
$classpath = $classes + [IO.Path]::PathSeparator + $PedroCoreJar
$sources = @()
foreach ($package in @('math', 'match', 'planning/pickup', 'state', 'turret', 'vision/pieces', 'scoring', 'robot')) {
    $sources += Get-ChildItem -LiteralPath (Join-Path $sourceRoot $package) -Filter '*.java' |
        Where-Object { $_.Name -notin @('HuskyLensPieceObservationSource.java', 'RevTurretIO.java', 'LimelightTargetSource.java', 'RevRobotIO.java', 'RobotHardwareConfig.java', 'RevBasicDrive.java', 'ShooterCalibrationStorage.java', 'ShotTrimStorage.java','HuskyLensEntryClassifier.java', 'DriveHardwareConfig.java', 'FlywheelPair.java') } |
        ForEach-Object FullName
}
# Simple Auto: only the pure-logic files. SimpleAutoStorage needs the Android SDK and is left out.
foreach ($name in @('SimpleAutoPlan', 'SimpleAutoRunner', 'FlywheelReadyDetector', 'HeadingHold')) {
    $sources += Join-Path $sourceRoot ('auto/' + $name + '.java')
}
foreach ($name in @('PedroManualDrive', 'PedroMappedReading', 'PedroPoseAdapter')) {
    $sources += Join-Path $sourceRoot ('pedro/' + $name + '.java')
}
$scenarios = @('PickupPlannerScenarios', 'PieceTrackerScenarios', 'RobotStateHistoryScenarios', 'TurretSafetyScenarios', 'PedroMappingScenarios', 'SafetyRegressionScenarios', 'TurretBenchScenarios', 'AuditRegressionScenarios', 'RobotIntegrationScenarios', 'MovingShotScenarios', 'LauncherCalibrationScenarios', 'CompetitionReadinessScenarios', 'VisionAimScenarios', 'AimPointScenarios', 'SimpleAutoScenarios', 'ShotTrimScenarios')
foreach ($scenario in $scenarios) {
    $sources += Join-Path $sourceRoot ('test/' + $scenario + '.java')
}
& $javac -encoding UTF-8 --release 8 -cp $PedroCoreJar -d $classes $sources
if ($LASTEXITCODE -ne 0) { throw "Desktop compilation failed (exit $LASTEXITCODE)." }
foreach ($scenario in $scenarios) {
    & $java -cp $classpath ('org.firstinspires.ftc.teamcode.test.' + $scenario)
    if ($LASTEXITCODE -ne 0) { throw "$scenario failed (exit $LASTEXITCODE)." }
}
Write-Output 'All desktop scenarios passed.'
