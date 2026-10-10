package igknighters.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.shooter.Hood;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterFlyWheel;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.turret.TurretNoAbstract;

public class intake {
    public intakePivot pivot = new intakePivot();
    public intakeRoller roller = new intakeRoller();


    // put the flywheels and the turret here
    /**
     * Targets the shooter to a specific state. Not a command.
     *
     * @param state
     */
    public void targetState(ShooterState state) {
        pivot.targetAngle(state.hoodAngle);
        roller.setSpeedNoCommand(state.turretAngle);
    }

    /**
     * Returns a command that targets the shooter to a specific state. This will not end internally
     *
     * @param state
     * @return
     */
    public Command targetStateCommand(ShooterState state) {
        return Commands.parallel(
                        hood.targetAngleCommand(state.hoodAngle),
                        turret.targetAngleCommand(state.turretAngle),
                        flyWheel.setSpeed(state.flywheelVelocity))
                .withName(
                        "TARGETING HOOD ANGLE: "
                                + state.hoodAngle.in(Degrees)
                                + ", TURRET ANGLE: "
                                + state.turretAngle.in(Degrees)
                                + ", RPM: "
                                + state.flywheelVelocity.in(RPM));
    }

    public ShooterState currentState(Shooter shooter) {
        return new ShooterState(
                shooter.hood.getCurrentAngle(),
                shooter.turret.getCurrentAngle(),
                shooter.flyWheel.getSpeed());
    }
    
}
