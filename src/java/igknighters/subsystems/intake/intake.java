package igknighters.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static IntakePivot pivot = new IntakePivot();
    public static IntakeRoller roller = new IntakeRoller();

    // put the flywheels and the turret here
    /**
     * Targets the shooter to a specific state. Not a command.
     *
     * @param state
     */
    public void targetState(IntakeState state) {
        pivot.targetAngle(state.pivotAngle);
        roller.setSpeedNoCommand(state.rollerSpeed);
    }

    /**
     * Returns a command that targets the shooter to a specific state. This will not end internally
     *
     * @param state
     * @return
     */
    public static Command targetStateCommand(IntakeState state) {
        return Commands.parallel(pivot.targetAngleCommand(state.pivotAngle))
                // roller.setSpeed(state.rollerSpeed))
                .withName(
                        "TARGETING PIVOT ANGLE: "
                                + state.pivotAngle.in(Degrees)
                                + ", ROLLER SPEED: "
                                + state.rollerSpeed.in(RPM));
    }

    public static IntakeState currentState(Intake intake) {
        return new IntakeState(intake.pivot.getCurrentAngle(), RPM.of(0.0));
    }
}
