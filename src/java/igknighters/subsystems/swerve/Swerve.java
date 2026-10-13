package igknighters.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.subsystems.swerve.swerveconstants.GeminiConsts;
import java.util.function.Consumer;
import java.util.function.Supplier;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem wrapper for the robot's swerve drive. This class wraps a {@link
 * CommandSwerveDrivetrain} and provides a consistent interface for driving, path following, and
 * vision integration.
 *
 * <p>It includes functionality to "disable" the swerve drive (using {@code isSwerveDisabled}),
 * which is useful for testing other subsystems without the drive motors being active.
 */
public class Swerve extends SubsystemBase {
    /** The actual CTRE-based swerve drivetrain. */
    CommandSwerveDrivetrain drivetrain;

    /** Flag to disable all drive motor outputs. */
    boolean isSwerveDisabled = false;

    /** A dummy swerve instance for "no-op" operations when disabled. */
    DummySwerve dummySwerve = new DummySwerve();

    /** Constructs a Swerve subsystem with the drive enabled. */
    public Swerve() {
        this(false);
    }

    public Orientation3d getRobotOrientation() {
        // need to test irl to see how the rotation is effected by mounting
        if (!isSwerveDisabled) {
            return new Orientation3d(drivetrain.getPigeon2().getRotation3d(), getAngularVelocity());
        } else {
            return new Orientation3d(new Rotation3d(), getAngularVelocity());
        }
    }

    public AngularVelocity3d getAngularVelocity() {
        if (!isSwerveDisabled) {
            // NOTE: IT IS UNKOWN AS TO WHETHER LL WANTS THE WORLD PERSPECTIVE OR ROBOT
            // ALSO I DO NOT KNOW IF I HAVE TO HANDLE THE WEIRD MOUNTING MANUALLY
            return new AngularVelocity3d(
                    drivetrain.getPigeon2().getAngularVelocityXDevice().getValue(),
                    drivetrain.getPigeon2().getAngularVelocityYDevice().getValue(),
                    drivetrain.getPigeon2().getAngularVelocityZDevice().getValue());
        } else {
            return new AngularVelocity3d(RPM.of(0), RPM.of(0), RPM.of(0));
        }
    }

    /**
     * Constructs a Swerve subsystem with an option to disable it.
     *
     * @param isSwerveDisabled True if the swerve drive should start disabled.
     */
    public Swerve(boolean isSwerveDisabled) {
        this.isSwerveDisabled = isSwerveDisabled;
        if (!isSwerveDisabled) {
            drivetrain = GeminiConsts.createDrivetrain(this);
        }
    }

    @Override
    public void periodic() {
        if (!isSwerveDisabled) {
            drivetrain.periodic();
            if (Robot.isRobotTest()) {
                Logger.recordOutput(
                        "ROBOT/TEST/SWERVE/CURRENT ROTATION DEGREES",
                        drivetrain.getPigeon2().getYaw().getValueAsDouble());
            }
        }
    }

    /**
     * Follows a single trajectory sample from Choreo.
     *
     * @param sample The {@link SwerveSample} to follow.
     */
    public void followPath(SwerveSample sample) {
        if (!isSwerveDisabled) {
            drivetrain.followPath(sample);
        }
    }

    /**
     * Resets the robot's odometry to the specified pose.
     *
     * @param pose The new {@link Pose2d} for the robot.
     */
    public void resetPose(Pose2d pose) {
        if (!isSwerveDisabled) {
            drivetrain.resetPose(pose);
        }
    }

    /**
     * Creates an {@link AutoFactory} for path planning and autonomous routines.
     *
     * @return A new AutoFactory instance.
     */
    public AutoFactory createAutoFactory() {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory();
        } else {
            return new AutoFactory(
                    () -> new Pose2d(), this::resetPose, this::followPath, true, this);
        }
    }

    /**
     * Creates an {@link AutoFactory} with a custom trajectory logger.
     *
     * @param logger A consumer that receives trajectory samples for logging.
     * @return A new AutoFactory instance.
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> logger) {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory(logger);
        } else {
            return new AutoFactory(
                    () -> new Pose2d(), this::resetPose, this::followPath, true, this, logger);
        }
    }

    /**
     * Returns a command that applies the given swerve request.
     *
     * @param requestSupplier A supplier for the {@link SwerveRequest} to apply.
     * @return A command representing the request.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if (!isSwerveDisabled) {
            return drivetrain.applyRequest(requestSupplier);
        } else {
            return dummySwerve.doNothing();
        }
    }

    /**
     * Returns a command for SysId quasistatic characterization.
     *
     * @param direction The direction to run the test.
     * @return The SysId quasistatic command.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if (!isSwerveDisabled) {
            return drivetrain.sysIdQuasistatic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }

    /**
     * Returns a command for SysId dynamic characterization.
     *
     * @param direction The direction to run the test.
     * @return The SysId dynamic command.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (!isSwerveDisabled) {
            return drivetrain.sysIdDynamic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }

    /**
     * Adds a vision measurement to the drivetrain's pose estimator.
     *
     * @param visionPose The robot pose as seen by vision.
     * @param timestamp The FPGA timestamp of the measurement.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        if (!isSwerveDisabled) {
            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }
    }

    /**
     * Adds a vision measurement with custom standard deviations.
     *
     * @param visionRobotPoseMeters The robot pose from vision.
     * @param timestampSeconds The FPGA timestamp.
     * @param visionMeasurementStdDevs Standard deviations for the measurement [x, y, theta].
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (!isSwerveDisabled) {
            drivetrain.addVisionMeasurement(
                    visionRobotPoseMeters,
                    Utils.fpgaToCurrentTime(timestampSeconds),
                    visionMeasurementStdDevs);
        }
    }

    /**
     * Returns the current state of the swerve drivetrain, including pose and module data.
     *
     * @return The current {@link SwerveDriveState}.
     */
    public SwerveDriveState getState() {
        if (!isSwerveDisabled) {
            return drivetrain.getState();
        } else {
            return new SwerveDriveState();
        }
    }

    /**
     * Returns the current field-relative speeds of the robot.
     *
     * @return The {@link ChassisSpeeds} relative to the field.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        if (!isSwerveDisabled) {
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        } else {
            return new ChassisSpeeds();
        }
    }

    /**
     * Registers a telemetry function that will be called periodically with the drive state.
     *
     * @param telemetryFunction The function to call.
     */
    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        if (!isSwerveDisabled) {
            drivetrain.registerTelemetry(telemetryFunction);
        }
    }

    /**
     * Applies a control request directly to the drivetrain.
     *
     * @param request The {@link SwerveRequest} to apply.
     */
    public void setControl(SwerveRequest request) {
        if (!isSwerveDisabled) {
            drivetrain.setControl(request);
        }
    }

    /**
     * Returns the maximum theoretical speed of the robot.
     *
     * @return Max speed in meters per second.
     */
    public double getMaxSpeedMetersPerSecond() {
        if (!isSwerveDisabled) {
            return GeminiConsts.kSpeedAt12Volts.in(MetersPerSecond);
        } else {
            return 0.0;
        }
    }

    /**
     * Returns the robot's current X acceleration from the IMU.
     *
     * @return Acceleration in Gs.
     */
    public double getXAcceleration() {
        if (!isSwerveDisabled) {
            return drivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    /**
     * Returns the robot's current Y acceleration from the IMU.
     *
     * @return Acceleration in Gs.
     */
    public double getYAcceleration() {
        if (!isSwerveDisabled) {
            return drivetrain.getPigeon2().getAccelerationY().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    /**
     * Returns the robot's current rotational velocity from the IMU.
     *
     * @return Angular velocity in radians per second.
     */
    public double getRotationalVelocity() {
        if (!isSwerveDisabled) {
            return drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()
                    * Conv.DEGREES_TO_RADIANS;
        } else {
            return 0.0;
        }
    }

    private AutoTrajectory activeTrajectory = null;
    private final Timer autoTimer = new Timer();

    /**
     * Sets the currently active autonomous trajectory and restarts the timer.
     *
     * @param trajectory The {@link AutoTrajectory} being followed.
     */
    public void setActiveTrajectory(AutoTrajectory trajectory) {
        this.activeTrajectory = trajectory;
        autoTimer.restart();
    }

    /** Clears the active trajectory and stops the timer. */
    public void clearActiveTrajectory() {
        this.activeTrajectory = null;
        autoTimer.stop();
    }

    /**
     * Returns the currently active autonomous trajectory.
     *
     * @return The active trajectory, or null if none.
     */
    public AutoTrajectory getActiveTrajectory() {
        return activeTrajectory;
    }

    /**
     * Returns the time elapsed since the start of the current autonomous trajectory.
     *
     * @return Time in seconds.
     */
    public double getAutoTime() {
        return autoTimer.get();
    }
}
