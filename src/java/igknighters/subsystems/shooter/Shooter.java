package igknighters.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.shooter.turret.TurretNoAbstract;

public class Shooter extends SubsystemBase {
    // shooter will hold all of the underlying mechanisms and it will be what is interfaced with
    public Hood hood = new Hood();
    public TurretNoAbstract turret = new TurretNoAbstract();

    // put the flywheels and the turret here
    /**
     * Targets the shooter to a specific state. Not a command.
     *
     * @param state
     */
    public void targetState(ShooterState state) {
        hood.targetAngleNoCommand(state.hoodAngle);
        turret.targetAngle(state.turretAngle);
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
                turret.targetAngleCommand(state.turretAngle));
    }

    public boolean isHoodSensorHit() {
        return hood.getHoodLimit();
    }
}
