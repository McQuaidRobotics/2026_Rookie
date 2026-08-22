package igknighters.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.shooter.Shooter;

public
class ShooterCommands { // tech for a real mech like for an og one but with yams the commands are in
    // class.
    public static Command setAngle(Shooter shooter, Angle angle) {
        return shooter.turret.run(() -> shooter.turret.targetAngle(angle));
    }

    public static Command faceForward(Shooter shooter) {
        return shooter.turret.run(
                () -> {
                    Pose2d robotPose = Robot.pose_pred.getPredictedPose();
                    Angle angle = robotPose.getRotation().getMeasure();
                    Angle targetAngle = angle.times(-1);
                    shooter.turret.targetAngle(targetAngle);
                });
    }
}
