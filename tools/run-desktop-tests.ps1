[CmdletBinding()]
param([string]$JdkHome)

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
$sources = @()
foreach ($package in @('math', 'match', 'planning/pickup', 'state', 'turret', 'vision/pieces')) {
    $sources += Get-ChildItem -LiteralPath (Join-Path $sourceRoot $package) -Filter '*.java' |
        Where-Object Name -ne 'HuskyLensPieceObservationSource.java' |
        ForEach-Object FullName
}
$scenarios = @('PickupPlannerScenarios', 'PieceTrackerScenarios', 'RobotStateHistoryScenarios', 'TurretSafetyScenarios')
foreach ($scenario in $scenarios) {
    $sources += Join-Path $sourceRoot ('test/' + $scenario + '.java')
}
& $javac -encoding UTF-8 --release 8 -d $classes $sources
if ($LASTEXITCODE -ne 0) { throw "Desktop compilation failed (exit $LASTEXITCODE)." }
foreach ($scenario in $scenarios) {
    & $java -cp $classes ('org.firstinspires.ftc.teamcode.test.' + $scenario)
    if ($LASTEXITCODE -ne 0) { throw "$scenario failed (exit $LASTEXITCODE)." }
}
Write-Output 'All desktop scenarios passed.'
