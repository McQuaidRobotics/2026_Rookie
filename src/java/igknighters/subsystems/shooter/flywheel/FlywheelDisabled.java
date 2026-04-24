package igknighters.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelDisabled extends Flywheel {

    @Override
    public AngularVelocity getSpeed() {
        return RPM.of(0);
    }
    @Override
    public void setSpeed(AngularVelocity speed) {
        // Do nothing
    }
    @Override
    public void periodic() {
        // Do nothing
    }
}
