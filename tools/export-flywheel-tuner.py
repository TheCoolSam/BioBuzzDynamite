"""Package the FTC flywheel bench tuner for importing into another Android Studio project."""

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def main():
    repo = Path(__file__).resolve().parents[1]
    teamcode = Path("TeamCode/src/main/java/org/firstinspires/ftc/teamcode")
    sources = (
        "test/TurretShooterBenchOpMode.java",
        "test/DashboardTunables.java",
        "scoring/FlywheelAutoTuner.java",
        "scoring/ShooterCalibrationStorage.java",
        "robot/FlywheelPair.java",
    )
    output = repo / "build/FTC-Flywheel-PIDF-Tuner.zip"
    output.parent.mkdir(parents=True, exist_ok=True)
    readme = (repo / "doc/FLYWHEEL_PIDF_FOR_TEAMS.md").read_text(encoding="utf-8")
    readme = readme.replace("](../TeamCode/", "](TeamCode/").replace("](../LICENSE)", "](LICENSE)")
    with ZipFile(output, "w", compression=ZIP_DEFLATED) as bundle:
        bundle.write(repo / "LICENSE", "LICENSE")
        bundle.writestr("README.md", readme)
        for source in sources:
            path = teamcode / source
            bundle.write(repo / path, path.as_posix())
    print(output)


if __name__ == "__main__":
    main()
