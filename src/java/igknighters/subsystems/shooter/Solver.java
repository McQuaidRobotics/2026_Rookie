package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.util.LerpTable.*;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants.kShooter;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.LerpTable;

public class Solver {
    public static LerpTable rpmTable = new LerpTable(
        new LerpTableEntry[] {
            new LerpTableEntry(0, 2000),
            new LerpTableEntry(10, 2500),
            new LerpTableEntry(20, 3000)
        }
    );
    public static LerpTable hoodTable = new LerpTable(
        new LerpTableEntry[] {
            new LerpTableEntry(0, 20),
            new LerpTableEntry(10, 25),
            new LerpTableEntry(20, 30)
        }
    );


    public static ShooterState solve (Pose2d pose, Pose2d target) {
        double distance = Math.sqrt(Math.pow(pose.getX() - target.getX(), 2) + Math.pow(pose.getY() - target.getY(), 2));

        double turretAngle = Math.atan((target.getY() - pose.getY()) / (target.getX() - pose.getX())) - pose.getRotation().getRadians();
        // NOTE KYLE U NEED TO ENSURE THESE ANGLES ARE POSSIBLE BC IF OUTSIDE OF LERP THEN ITS NOT POSIBLE
        double hoodAngleDeg = hoodTable.lerp(distance);
        double rpm = rpmTable.lerp(distance);

        if (hoodAngleDeg < kHood.MIN_ANGLE_DEGREES || hoodAngleDeg > kHood.MAX_ANGLE_DEGREES || rpm < 0 || rpm > kShooter.kFlywheels.MAX_SPEED_RPM) {
            ShootInformation.getInstance().setPossibleShot(false);
            return new ShooterState(Degrees.of(0.0), Radians.of(turretAngle), RPM.of(0)); // returning 0 means a shot is not possible
        }
        return new ShooterState(Degrees.of(hoodTable.lerp(distance)), Radians.of(turretAngle), RPM.of(rpmTable.lerp(distance)));
    }
}
