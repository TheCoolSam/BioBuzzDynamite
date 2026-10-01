package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.turret.TurretCommand;
import org.firstinspires.ftc.teamcode.turret.TurretController;
import org.firstinspires.ftc.teamcode.turret.TurretState;
import org.firstinspires.ftc.teamcode.turret.TurretStateAdapter;

/** Desktop regression checks for failures at the state-to-controller boundary. */
public final class TurretSafetyScenarios {
    private static int checks;

    public static void main(String[] args) {
        RobotState robot = new RobotState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, true);
        for (double bad : new double[] { Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY }) {
            hold(TurretStateAdapter.toTurretState(robot, bad, 0, 0, 100, 0.02), "encoder angle");
            hold(TurretStateAdapter.toTurretState(robot, 0, bad, 0, 100, 0.02), "encoder velocity");
            hold(TurretStateAdapter.toTurretState(robot, 0, 0, bad, 100, 0.02), "target x");
            hold(TurretStateAdapter.toTurretState(robot, 0, 0, 0, bad, 0.02), "target y");
            hold(TurretStateAdapter.toTurretState(robot, 0, 0, 0, 100, bad), "loop time");
        }
        hold(TurretStateAdapter.toTurretState(null, 0, 0, 0, 100, 0.02), "missing robot");
        hold(TurretStateAdapter.toTurretState(RobotState.invalid(0), 0, 0, 0, 100, 0.02), "invalid pose");
        TurretController controller = new TurretController();
        TurretCommand valid = controller.calculate(TurretStateAdapter.toTurretState(robot, 0, 0, 0, 100, 0.02));
        check(valid.tracking && valid.turretMotorPower > 0, "valid measurements still aim");
        TurretCommand failure = controller.calculate(TurretStateAdapter.toTurretState(robot, Double.NaN, 0, 0, 100, 0.02));
        check(!failure.tracking && failure.turretMotorPower == 0, "failure after active tracking stops power");
        RobotState badVelocity = new RobotState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, false);
        TurretCommand withoutFeedforward = controller.calculate(
                TurretStateAdapter.toTurretState(badVelocity, 0, 0, 0, 100, 0.02));
        check(withoutFeedforward.tracking, "valid pose can aim without robot velocity");
        check(withoutFeedforward.desiredTurretVelocity == 0, "invalid robot velocity drops feedforward");
        System.out.println("TURRET SAFETY CHECKS PASSED (" + checks + ")");
    }

    private static void hold(TurretState state, String label) {
        TurretCommand command = new TurretController().calculate(state);
        check(!command.tracking, label + " refuses tracking");
        check(command.turretMotorPower == 0, label + " has zero motor power");
        check(command.requestedChassisOmega == 0, label + " has zero chassis request");
        check(Double.isFinite(command.desiredTurretAngle), label + " publishes a finite hold angle");
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }
}
