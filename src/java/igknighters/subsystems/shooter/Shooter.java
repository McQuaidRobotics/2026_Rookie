package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
    // KYLE : ONCE THIS GOES INTO UR FLYWHEEL BRANCH YOU ARE GOING TO REPLACE THIS WITH A WORKING THING
    public AngularVelocity getFlywheelVelocity() {
        return RPM.of(0.0);
        // return flywheel.getFlywheelVelocity(); 
    }

    public Angle getHoodAngle() {
        return hood.getCurrentAngle();
    }
    public Angle getTurretAngle() {
        return turret.getCurrentAngle();
    }
    public boolean isHoodSensorHit() {
        return hood.getHoodLimit();
    }
}
