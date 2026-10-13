package igknighters.subsystems.LimeLightVision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import igknighters.subsystems.LimeLightVision.Cameras.YallLimelight;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import limelight.networktables.LimelightSettings.ImuMode;

public class LimeLightVision extends SubsystemBase {

    public record pose_output(Pose2d pose, Time time) {}

    public record tag_output(Translation3d tag_info, Time time) {}

    public record object_output(ArrayList<Translation3d> object_info, Time time) {}

    private final ArrayList<YallLimelight> cameras = new ArrayList<>();

    public Time latestMeasurementTime = Seconds.of(0.0);

    public LimeLightVision() {
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                SubsystemConstants.kLimelightVision.frontCam,
                                SubsystemConstants.kLimelightVision.frontCamPose,
                                Pipelines.POSE_DETECTION,
                                false)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                SubsystemConstants.kLimelightVision.backCam,
                                SubsystemConstants.kLimelightVision.backCamPose,
                                Pipelines.POSE_DETECTION,
                                false)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                SubsystemConstants.kLimelightVision.primaryCam,
                                SubsystemConstants.kLimelightVision.primaryCamPose,
                                Pipelines.POSE_DETECTION,
                                false)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                SubsystemConstants.kLimelightVision.leftCam,
                                SubsystemConstants.kLimelightVision.leftCamPose,
                                Pipelines.POSE_DETECTION,
                                false)));
    }

    public object_output getObjectInfo(String objectName, double confidence) {
        ArrayList<Translation3d> allDetectedObjects = new ArrayList<>();
        Time latestTime = Seconds.of(0.0);

        for (YallLimelight camera : cameras) {
            if (camera.data.cameraPipeline.equals(Pipelines.OBJECT_DETECTION)
                    || camera.data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)) {

                object_output result = camera.getObjectTranslation(objectName, confidence);
                if (result != null && result.object_info() != null) {
                    allDetectedObjects.addAll(result.object_info());
                    if (result.time.gt(latestTime)) {
                        latestTime = result.time();
                    }
                }
            }
        }
        return new object_output(allDetectedObjects, latestTime);
    }

    public List<Integer> getVisibleTagIds() {
        LinkedHashSet<Integer> visibleTagIds = new LinkedHashSet<>();
        for (YallLimelight camera : cameras) {
            visibleTagIds.addAll(camera.getVisibleTagIds());
        }
        return new ArrayList<>(visibleTagIds);
    }

    public void enableCameras(ImuMode IMU_MODE) {
        for (YallLimelight camera : cameras) {
            camera.setIMUMode(IMU_MODE);
            camera.setThrottle(0);
        }
    }

    public void disableCameras() {
        for (YallLimelight camera : cameras) {
            camera.setThrottle(50);
        }
    }

    public double getLastTimeStamp() {
        return latestMeasurementTime.in(Seconds);
    }

    public double timeSinceLastSample() {
        return (RobotController.getFPGATime() / 1e6) - latestMeasurementTime.in(Seconds);
    }

    public List<pose_output> getRobotPoseFromVision() {
        if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
            Log.log("ROBOT/Subsystems/Vison/Limelight/ENABLED", true);
        }
        ArrayList<pose_output> outputs = new ArrayList<>();
        Time max = Seconds.of(0.0);

        for (YallLimelight camera : cameras) {
            if (camera.data.cameraPipeline.equals(Pipelines.POSE_DETECTION)
                    || camera.data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)) {

                pose_output output = camera.getRobotPoseFromVision();
                if (output == null) {
                    continue;
                }
                if (output.time.gt(max)) {
                    max = output.time();
                }
                outputs.add(output);
            }
        }

        if (latestMeasurementTime.lt(max)) {
            latestMeasurementTime = max;
        }
        return outputs;
    }

    @Override
    public void periodic() {
        for (YallLimelight limelight : cameras) {
            limelight.periodic();
        }
        // one flush for all cameras instead of two per camera
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void simulationPeriodic() {
        for (YallLimelight limelight : cameras) {
            limelight.simPeriodic(Robot.pose_pred.getDynamicPredictedPose());
        }
    }
}
