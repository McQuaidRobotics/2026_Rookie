package igknighters.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterState {
    public Angle hoodAngle;
    public Angle turretAngle;
    public AngularVelocity flywheelVelocity;

    public ShooterState(Angle hoodAngle, Angle turretAngle, AngularVelocity flywheelVelocity) {
        this.hoodAngle = hoodAngle;
        this.turretAngle = turretAngle;
        this.flywheelVelocity = flywheelVelocity;
    }
}
