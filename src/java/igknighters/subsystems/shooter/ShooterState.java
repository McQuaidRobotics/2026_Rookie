package igknighters.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;

public class ShooterState {
    public Angle turretAngle;

    public ShooterState(Angle turretAngle) {
        this.turretAngle = turretAngle;
    }
}
