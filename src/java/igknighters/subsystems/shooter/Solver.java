package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.turret.TurretNoAbstract;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.*;
import igknighters.util.log.Log;

public class Solver {
    public static LerpTable rpmTable =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.5, 2700),
                        new LerpTableEntry(2.0, 2800),
                        new LerpTableEntry(2.5, 2900),
                        new LerpTableEntry(3.5, 3200),
                        new LerpTableEntry(3.8, 3220),
                        new LerpTableEntry(4.0, 3300),
                        new LerpTableEntry(4.5, 3400),
                        new LerpTableEntry(5.2, 3680),
                        new LerpTableEntry(5.5, 3780),
                        new LerpTableEntry(6.0, 3800),
                        new LerpTableEntry(8.0, 4000),
                        new LerpTableEntry(10.0, 4200),
                        new LerpTableEntry(20, 5500)
                    });
    public static LerpTable hoodTable =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(0, 18.6),
                        new LerpTableEntry(1, 19),
                        new LerpTableEntry(2, 25),
                        new LerpTableEntry(3, 30),
                        new LerpTableEntry(4, 35),
                        new LerpTableEntry(5, 40),
                        new LerpTableEntry(5.6, 38),
                        new LerpTableEntry(6, 45),
                    });

    public static ShooterState solve(Pose2d pose, Pose2d target) {
        double distance =
                Math.sqrt(
                        Math.pow(pose.getX() - target.getX(), 2)
                                + Math.pow(pose.getY() - target.getY(), 2));
        Log.log("ROBOT/SUBSYSTEMS/SHOOTER/DISTANCE", distance);

        double turretAngle =
                Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX())
                        - pose.getRotation().getRadians();
        // NOTE KYLE U NEED TO ENSURE THESE ANGLES ARE POSSIBLE BC IF OUTSIDE OF LERP THEN ITS NOT
        // POSIBLE
        double hoodAngleDeg = hoodTable.lerp(distance);
        double rpm = rpmTable.lerp(distance);

        if (hoodAngleDeg < kHood.MIN_ANGLE_DEGREES
                || hoodAngleDeg > kHood.MAX_ANGLE_DEGREES
                || rpm < 0
                || rpm > kShooter.kFlywheels.MAX_SPEED_RPM) {
            ShootInformation.getInstance().setPossibleShot(false);
            return new ShooterState(
                    Degrees.of(0.0),
                    Radians.of(TurretNoAbstract.wrapAngle(Radians.of(turretAngle)).in(Radians)),
                    RPM.of(0)); // returning 0 means a shot is not possible
        }
        return new ShooterState(
                Degrees.of(hoodTable.lerp(distance)),
                Radians.of(TurretNoAbstract.wrapAngle(Radians.of(turretAngle)).in(Radians)),
                RPM.of(rpmTable.lerp(distance)));
    }
}
