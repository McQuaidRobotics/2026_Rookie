package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableBoolean;
import java.util.List;

/**
 * FieldVisualizer provides methods to update specific objects on the field (Field2d) independently
 * of the main telemetry loop. This allows for updating vision targets, detected objects, and other
 * field elements only when necessary.
 *
 * <p>It uses a singleton pattern to ensure a single entry point for field visualization updates
 * across different commands and subsystems.
 */
public class FieldVisualizer {

    private FieldVisualizer() {
        fieldTypePub.set("Field2d");
    }

    private static class SingletonHelper {
        private static final FieldVisualizer INSTANCE = new FieldVisualizer();
    }

    /**
     * Returns the singleton instance of the FieldVisualizer.
     *
     * @return The FieldVisualizer instance.
     */
    public static FieldVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }

    private final TunableBoolean shouldShowBalls =
            TunableValues.getBoolean("FieldVisualizer/ShowBalls", true);
    private final TunableBoolean shouldShowShootingTarget =
            TunableValues.getBoolean("FieldVisualizer/ShowShootingTarget", true);
    private final TunableBoolean shouldShowDrivingTarget =
            TunableValues.getBoolean("FieldVisualizer/ShowDrivingTarget", true);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final DoubleArrayPublisher shootingTargetPub =
            table.getDoubleArrayTopic("shootingTargetPose").publish();

    public final Pose2d robotPose = new Pose2d();

    private final DoubleArrayPublisher turretAnglePub =
            table.getDoubleArrayTopic("turretAngle").publish();

    private final DoubleArrayPublisher predictedFuturePose =
            table.getDoubleArrayTopic("futurePose").publish();

    private final DoubleArrayPublisher drivingTargetPub =
            table.getDoubleArrayTopic("drivingTargetPose").publish();

    private final DoubleArrayPublisher detectedObjectsPub =
            table.getDoubleArrayTopic("detectedObjects").publish();

    /**
     * Updates the shooting target pose on the field.
     *
     * @param target The pose of the shooting target, or null to clear.
     */
    public void updateShootingTarget(Pose2d target) {
        if (target == null || !shouldShowShootingTarget.value()) {
            shootingTargetPub.set(new double[0]);
            return;
        }
        shootingTargetPub.set(
                new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
    }

    /**
     * Updates the predicted future pose of the robot on the field.
     *
     * @param pred_pose The predicted {@link Pose2d}.
     */
    public void updatePredictedPose(Pose2d pred_pose) {
        if (pred_pose == null) {
            predictedFuturePose.set(new double[0]);
            return;
        }
        predictedFuturePose.set(
                new double[] {
                    pred_pose.getX(), pred_pose.getY(), pred_pose.getRotation().getDegrees()
                });
    }

    /**
     * Updates the driving target pose on the field.
     *
     * @param target The pose of the driving target, or null to clear.
     */
    public void updateDrivingTarget(Pose2d target) {
        if (target == null || !shouldShowDrivingTarget.value()) {
            drivingTargetPub.set(new double[0]);
            return;
        }
        drivingTargetPub.set(
                new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
    }

    /**
     * Updates the turret orientation visualization relative to the robot's pose.
     *
     * @param turretAngleDegrees The current rotation of the turret.
     * @param robotPose The robot's current {@link Pose2d}.
     */
    public void updateTurret(double turretAngleDegrees, Pose2d robotPose) {
        Pose2d newPose =
                new Pose2d(
                        robotPose.getX(),
                        robotPose.getY(),
                        robotPose
                                .getRotation()
                                .plus(new Rotation2d(Math.toRadians(turretAngleDegrees))));
        turretAnglePub.set(
                new double[] {newPose.getX(), newPose.getY(), newPose.getRotation().getDegrees()});
    }

    /**
     * Updates the list of detected objects on the field.
     *
     * @param objects A list of poses for detected objects, or null/empty to clear.
     */
    public void updateDetectedObjects(List<Pose2d> objects) {
        if (objects == null || objects.isEmpty() || !shouldShowBalls.value()) {
            detectedObjectsPub.set(new double[0]);
            return;
        }
        int i = 0;
        double[] array = new double[objects.size() * 3];
        for (Pose2d obj : objects) {
            array[i++] = obj.getX();
            array[i++] = obj.getY();
            array[i++] = obj.getRotation().getDegrees();
        }
        detectedObjectsPub.set(array);
    }

    /**
     * Updates all vision-related targets on the field in one call.
     *
     * @param detectedObjects List of detected object poses.
     * @param shootingTarget Shooting target pose.
     * @param drivingTarget Driving target pose.
     */
    public void updateVisionTargets(
            List<Pose2d> detectedObjects, Pose2d shootingTarget, Pose2d drivingTarget) {
        updateDetectedObjects(detectedObjects);
        updateShootingTarget(shootingTarget);
        updateDrivingTarget(drivingTarget);
    }
}
