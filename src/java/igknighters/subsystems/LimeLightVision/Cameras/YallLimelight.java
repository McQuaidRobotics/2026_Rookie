package igknighters.subsystems.LimeLightVision.Cameras;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.CameraData;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import igknighters.subsystems.LimeLightVision.LimeLightVision.object_output;
import igknighters.subsystems.LimeLightVision.LimeLightVision.pose_output;
import igknighters.subsystems.LimeLightVision.LimeLightVision.tag_output;
import java.util.ArrayList;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightData;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightUtils;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralDetector;
import limelight.results.RawFiducial;
import limelight.sim.LimelightSim;

public class YallLimelight {
    public CameraData data;
    public Limelight camera;
    public LimelightSim sim_camera;
    public LimelightPoseEstimator poseEstimator;
    public boolean rotation_modified = false;
    public Pipelines functioning_as_pipeline = Pipelines.POSE_DETECTION;

    public final ArrayList<Integer> visible_tag_ids = new ArrayList<>();

    // Same NT topic as LimelightSettings.withRobotOrientation, but without its per-call flush.
    // LimeLightVision.periodic flushes once for all cameras.
    private final DoubleArrayPublisher robotOrientationPub;

    // Offset: (RIO FPGA Time) - (Limelight Hardware Time)
    public Time time_offset = Seconds.of(0);
    public boolean is_first_reading = true;

    public YallLimelight(CameraData data) {
        this.data = data;
        camera = new Limelight(data.name);
        sim_camera = new LimelightSim(camera);
        robotOrientationPub =
                camera.getNTTable().getDoubleArrayTopic("robot_orientation_set").publish();

        if (data.use_nt_position) {
            // stick with internal config
            camera.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).save();
        } else {
            // apply offset
            camera.getSettings()
                    .withLimelightLEDMode(LEDMode.PipelineControl)
                    .withCameraOffset(data.cameraFloorRobotCenter)
                    .save();
            sim_camera.withRobotToCameraTransform(
                    new Transform3d(
                            data.cameraFloorRobotCenter.getTranslation(),
                            data.cameraFloorRobotCenter.getRotation()));
        }
        poseEstimator = camera.createPoseEstimator(EstimationMode.MEGATAG2);

        switch (data.cameraPipeline) {
            case POSE_DETECTION:
                camera.getSettings().withPipelineIndex(1).save();
                break;
            case OBJECT_DETECTION:
                camera.getSettings().withPipelineIndex(2).save();
                break;
            case TAG_TRACKING:
                camera.getSettings().withPipelineIndex(3).save();
                break;
            case DOES_EVERYTHING:
                break;
            default:
                DriverStation.reportWarning(
                        "THE CAMERA NAMED: "
                                + data.name
                                + " is not configured to be any known pipeline",
                        true);
                break;
        }
    }

    /**
     * Calibrates the difference between RIO FPGATime and the Limelight hardware clock on the very
     * first valid frame received.
     */
    private synchronized void synchronizeClock(Time llHardwareTimestamp) {
        if (is_first_reading) {
            Time rioFpgaTime = Microseconds.of(RobotController.getFPGATime());
            time_offset = rioFpgaTime.minus(llHardwareTimestamp);
            is_first_reading = false;
        }
    }

    /** Converts a raw Limelight timestamp to the RIO FPGA timeline using the stored offset. */
    private Time toRioFpgaTime(Time llHardwareTimestamp) {
        return llHardwareTimestamp.plus(time_offset);
    }

    public double calculateScore(double ambiguity, double distanceToRobot) {
        return (2 - (ambiguity + (distanceToRobot / 4.0))) / 2.0;
    }

    public void setThrottle(double throttle) {
        camera.getSettings().withThrottle(throttle).save();
    }

    public void setIMUMode(ImuMode imuMode) {
        camera.getSettings().withImuMode(imuMode).save();
    }

    public pose_output getRobotPoseFromVision() {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                || data.cameraPipeline.equals(Pipelines.POSE_DETECTION))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR A POSE FROM A NON POSE DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.POSE_DETECTION)) {
            functioning_as_pipeline = Pipelines.POSE_DETECTION;
            camera.getSettings().withPipelineIndex(1).save();
        }

        if (data.cameraOffsetFromAxisOfRotation != null && !rotation_modified) {
            DriverStation.reportWarning(
                    "YOU ARE NOT SUPPLYING A ROTATION TO A ROTATING CAMERA THIS WILL MESS UP VISION"
                            + " MEASUREMENTS",
                    true);
            return null;
        }

        Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
        if (visionEstimate.isPresent()) {
            PoseEstimate validEstimate = visionEstimate.get();

            if (validEstimate.avgTagDist < 4
                    && validEstimate.tagCount > 1
                    && validEstimate.getMinTagAmbiguity() < 0.3) {

                Time llTimestamp = Seconds.of(validEstimate.timestampSeconds);
                synchronizeClock(llTimestamp);

                return new pose_output(validEstimate.pose.toPose2d(), toRioFpgaTime(llTimestamp));
            }
        }
        return null;
    }

    public ArrayList<Integer> getVisibleTagIds() {
        return visible_tag_ids;
    }

    public tag_output getTagTranslation(Integer tag_id, double pickyness) {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                || data.cameraPipeline.equals(Pipelines.TAG_TRACKING))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR TAG INFO FROM A NON TAG DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.TAG_TRACKING)) {
            functioning_as_pipeline = Pipelines.TAG_TRACKING;
            camera.getSettings().withPipelineIndex(3).save();
        }

        LimelightData limelightData = camera.getData();
        Optional<LimelightResults> potentialResults = camera.getLatestResults();
        if (!potentialResults.isPresent()) {
            return null;
        }

        LimelightResults results = potentialResults.get();
        Time llTimestamp = Milliseconds.of(results.timestamp_LIMELIGHT_publish);
        synchronizeClock(llTimestamp);

        for (RawFiducial tag : limelightData.getRawFiducials()) {
            if (tag.id == tag_id) {
                if (calculateScore(tag.ambiguity, tag.distToCamera) >= pickyness) {
                    return new tag_output(
                            new Translation3d(tag.txnc, tag.tync, tag.ta),
                            toRioFpgaTime(llTimestamp));
                }
            }
        }
        return null;
    }

    public object_output getObjectTranslation(String objectName, double required_confidence) {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                || data.cameraPipeline.equals(Pipelines.OBJECT_DETECTION))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR OBJECT INFO FROM A NON OBJECT DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.OBJECT_DETECTION)) {
            functioning_as_pipeline = Pipelines.OBJECT_DETECTION;
            camera.getSettings().withPipelineIndex(2).save();
        }

        Optional<LimelightResults> potential_results = camera.getLatestResults();
        if (!potential_results.isPresent()) {
            return null;
        }

        LimelightResults results = potential_results.get();
        Time llTimestamp = Milliseconds.of(results.timestamp_LIMELIGHT_publish);
        synchronizeClock(llTimestamp);

        ArrayList<Translation3d> outputs = new ArrayList<>();
        for (NeuralDetector object : results.targets_Detector) {
            if (object.className.equals(objectName) && object.confidence > required_confidence) {
                outputs.add(new Translation3d(object.tx, object.ty, object.ta));
            }
        }

        if (outputs.isEmpty()) {
            return null;
        } else {
            return new object_output(outputs, toRioFpgaTime(llTimestamp));
        }
    }

    private void handleTagVisibility() {
        RawFiducial[] potentialResults = camera.getData().getRawFiducials();
        if (potentialResults == null || potentialResults.length == 0) {
            visible_tag_ids.clear();
            return;
        }

        for (RawFiducial tag : potentialResults) {
            if (!visible_tag_ids.contains(tag.id)) {
                visible_tag_ids.add(tag.id);
            }
        }
    }

    /** Publishes orientation (and moving-camera offset) without flushing; caller flushes. */
    public void periodic() {
        if (data.orientationSupplier != null
                && data.cameraOffsetFromAxisOfRotation != null
                && !data.use_nt_position) {
            rotation_modified = true;
            camera.getSettings()
                    .withCameraOffset(
                            new Pose3d(
                                    data.cameraOffsetFromAxisOfRotation.getTranslation(),
                                    data.orientationSupplier.get()));
        }
        robotOrientationPub.set(LimelightUtils.orientation3dToArray(Robot.robotOrientation));
        handleTagVisibility();
    }

    public void simPeriodic(Pose2d robotPose) {
        sim_camera.update(robotPose);
    }
}
