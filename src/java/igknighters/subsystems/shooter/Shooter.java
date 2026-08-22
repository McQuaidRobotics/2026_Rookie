package igknighters.subsystems.shooter;

import igknighters.subsystems.shooter.turret.TurretNoAbstract;

public class Shooter {
    public TurretNoAbstract turret = new TurretNoAbstract();

    public void targetState(ShooterState state) {
        turret.targetAngle(state.turretAngle);
    }
}
