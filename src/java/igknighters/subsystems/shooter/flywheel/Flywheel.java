package igknighters.subsystems.shooter.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;

public abstract class Flywheel {
    public abstract void setSpeed(AngularVelocity speed);
    public abstract AngularVelocity getSpeed();
    public abstract void periodic();

    
}
