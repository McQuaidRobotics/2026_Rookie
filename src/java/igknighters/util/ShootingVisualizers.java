package igknighters.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import igknighters.util.log.Log;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ShootingVisualizers {
    /**
     * Visualizes the trajectory of a shot fired from the given position and angles.
     *
     * @param shooterPose The position and orientation of the shooter (robot pose + shooter offset)
     * @param robotSpeeds The field-relative speeds of the robot
     * @param launchAngle The angle at which the shot is launched (0 = horizontal, 90 deg = vertical)
     * @param turretAngle The angle of the turret (0 = facing forward on the robot)
     * @param launchVelocity The velocity at which the shot is launched
     * @param outputName The AdvantageKit log key (e.g., "ShotTrajectory")
     * @param fidelity The total number of points to calculate along the arc
     * @param heightCutoff The minimum height (z in meters) at which to stop calculating
     */
    public static void visualizeShotTrajectory(
            Pose3d shooterPose,
            ChassisSpeeds robotSpeeds,
            Angle launchAngle,
            Angle turretAngle,
            LinearVelocity launchVelocity,
            String outputName,
            int fidelity,
            double heightCutoff) {

        final double g = 9.80665;

        // 1. Calculate Field-Relative Turret Heading
        double robotYaw = shooterPose.getRotation().getZ();
        double fieldTurretHeading = robotYaw + turretAngle.in(Radians);

        // 2. Calculate Launch Velocity Components (Shot Frame)
        double speed = launchVelocity.in(MetersPerSecond);
        double v_horiz = speed * Math.cos(launchAngle.in(Radians));
        double v_z_launch = speed * Math.sin(launchAngle.in(Radians));

        // 3. Vector addition: Add Field-Relative Robot Speeds
        double v_x = (v_horiz * Math.cos(fieldTurretHeading)) + robotSpeeds.vxMetersPerSecond;
        double v_y = (v_horiz * Math.sin(fieldTurretHeading)) + robotSpeeds.vyMetersPerSecond;
        double v_z = v_z_launch;

        double z0 = shooterPose.getZ();

        // 4. Solve for quadratic roots where z(t) = heightCutoff
        // 0.5*g*t^2 - v_z*t + (heightCutoff - z0) = 0
        double a = 0.5 * g;
        double b = -v_z;
        double c = heightCutoff - z0;

        double discriminant = b * b - 4 * a * c;

        // If discriminant < 0, shot never reaches heightCutoff
        if (discriminant < 0) {
            Log.log("ROBOT/SUBSYSTEMS/SHOOTER/FAILURE REASON", "Shot never reaches height cutoff");
            Logger.recordOutput(outputName, new Pose3d[0]);
            return;
        }

        double sqrtDisc = Math.sqrt(discriminant);
        double tEnd = (-b + sqrtDisc) / (2 * a); // Second collision (falling)


        double tStart = 0.0;

        // If the second collision is in the past, nothing to show
        if (tEnd <= tStart) {
            Logger.recordOutput(outputName, new Pose3d[0]);
            return;
        }

        // 5. Sample trajectory points between tStart and tEnd
        List<Pose3d> trajectory = new ArrayList<>();
        double duration = tEnd - tStart;
        double dt = duration / Math.max(1, fidelity - 1);

        double startX = shooterPose.getX();
        double startY = shooterPose.getY();

        for (int i = 0; i < fidelity; i++) {
            double t = tStart + (i * dt);

            double x = startX + v_x * t;
            double y = startY + v_y * t;
            double z = z0 + (v_z * t) - (0.5 * g * t * t);

            // Orient point along direction of travel
            double currentVz = v_z - (g * t);
            double currentHorizSpeed = Math.hypot(v_x, v_y);
            double currentPitch = Math.atan2(currentVz, currentHorizSpeed);
            double currentYaw = Math.atan2(v_y, v_x);

            Rotation3d pointRotation = new Rotation3d(0, -currentPitch, currentYaw);
            trajectory.add(new Pose3d(new Translation3d(x, y, z), pointRotation));
        }

        // 6. Log trajectory to AdvantageKit
        Logger.recordOutput(outputName, trajectory.toArray(new Pose3d[0]));
    }

    public static void clear_shot_trajectory(String name_to_clear) {
        Logger.recordOutput(name_to_clear, new Pose3d[0]);
    }
}
