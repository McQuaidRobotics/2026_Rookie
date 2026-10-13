package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

public class CameraData {
    public Pose3d cameraFloorRobotCenter;
    public Transform3d cameraOffsetFromAxisOfRotation;
    public Pipelines cameraPipeline;
    public String name;
    public Supplier<Rotation3d> orientationSupplier;
    public boolean use_nt_position = false;

    public enum Pipelines {
        OBJECT_DETECTION, // output coords of game piece in field map
        POSE_DETECTION, // output coords of robot in field map
        TAG_TRACKING, // output simply a tx, ty and a
        DOES_EVERYTHING; // Will not complain if you ask it to do anything

        // IF YOU CAN THINK OF A BETTER WAY TO DO THIS IM ALL EARS BUT MY PLAN IS
        // IDX 1: Pose detection
        // IDX 2: Object detection
        // IDX 3: Tag Tracking

        // this would entail when configuring the limelights with our pipelines we put corresponding
        // pipelines on the corresponding idxs
        // So when you ask a camera to do pose detection it will use idx 1. The do everything is a
        // little niche but i was thinking a single camera
        // might be usefull doing different things at different times. I do not forsee switching
        // rapidly to be a good idea however it could do something
        // different begining during and end differently. Kinda niche but i think it has use.

    }

    /**
     * static camera parameters
     *
     * @param name
     * @param cameraFloorRobotCenter
     * @param cameraPipeline
     */
    public CameraData(
            String name,
            Pose3d cameraFloorRobotCenter,
            Pipelines cameraPipeline,
            boolean use_nt_position) {
        this.name = name;
        this.cameraFloorRobotCenter = cameraFloorRobotCenter;
        this.cameraPipeline = cameraPipeline;
        this.cameraOffsetFromAxisOfRotation = null;
        this.use_nt_position = use_nt_position;
    }

    /**
     * moving camera parameters
     *
     * @param name
     * @param cameraFloorRobotCenter
     * @param cameraOffsetFromRotation
     * @param cameraPipeline
     */
    public CameraData(
            String name,
            Pose3d cameraFloorRobotCenter,
            Transform3d cameraOffsetFromRotation,
            Supplier<Rotation3d> orientationSupplier,
            Pipelines cameraPipeline) {
        this.name = name;
        this.cameraFloorRobotCenter = cameraFloorRobotCenter;
        this.cameraPipeline = cameraPipeline;
        this.cameraOffsetFromAxisOfRotation = cameraOffsetFromRotation;
        this.orientationSupplier = orientationSupplier;
    }
}
