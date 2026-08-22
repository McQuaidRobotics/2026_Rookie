// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.Conv;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.constants.SubsystemConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.FuelSim;
import igknighters.util.RobotPosePredError;
import igknighters.util.RobotPosePredictor;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.TurretPosePredError;
import igknighters.util.TurretPosePredictor;
import igknighters.util.log.Log;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The main robot class that extends LoggedRobot for AdvantageKit integration. This class handles
 * the robot's lifecycle, subsystem initialization, command binding, and periodic updates.
 *
 * <p>It serves as the central hub for all robot functionality and coordination between subsystems
 * and commands.
 */
public class Robot extends LoggedRobot {

    // --- Commands & Scheduling ---
    private Command m_autonomousCommand;
    private AutoFactory autoFactory;

    /** Chooser for autonomous routines. */
    public final AutoChooser autoChooser = new AutoChooser();

    /** Chooser for test routines. */
    public final AutoChooser testChooser = new AutoChooser();

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();

    // --- State Predictors & Logging ---
    /** Global robot pose predictor. */
    public static RobotPosePredictor pose_pred;

    /** Predictor for turret pose relative to the field. */
    public static TurretPosePredictor turret_pred = new TurretPosePredictor();

    /** Error tracking for pose prediction. */
    public static RobotPosePredError pose_pred_error = new RobotPosePredError();

    /** Error tracking for turret pose prediction. */
    public static TurretPosePredError turret_pred_error = new TurretPosePredError();

    private Telemetry logger;

    // --- Controllers & Subsystems ---
    private final DriverController driverController = new DriverController(0);

    /** Central container for all robot subsystems. */
    public final Subsystems subsystems;

    // --- Simulation ---
    private FuelSim fuelSim;
    private double lastShotTime = 0.0;

    // --- Configuration & Tunables ---
    private final boolean kUseLimelight = true;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 1.0);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    /**
     * Configures logging for command lifecycle events (Initialize, Interrupt, Finish). This helps
     * in debugging command behavior through logs.
     */
    public void setUpCommandLogging() {
        if (!SubsystemConstants.disableAllLogs) {
            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "TRUE"));

            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));

            scheduler.onCommandInterrupt(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "TRUE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "FALSE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));
        }
    }

    /**
     * Publishes command and subsystem data to SmartDashboard for visibility.
     *
     * @param subsystems The robot subsystems to publish.
     */
    public void publishCommandsAndSubystems(Subsystems subsystems) {
        SmartDashboard.putData(CommandScheduler.getInstance());
        for (var subsystem : subsystems.lockedResources) {
            SmartDashboard.putData("SubsystemCommands/" + subsystem.getName(), subsystem);
        }
    }

    /**
     * Initializes the autonomous routine factories and choosers.
     *
     * @param subsystems The robot subsystems needed for auto factory creation.
     */
    public void setUpAutos(Subsystems subsystems) {
        autoFactory = subsystems.swerve.createAutoFactory();

        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
        SmartDashboard.putData("TEST CHOOSER", testChooser);
    }

    /**
     * Configures the swerve drive subsystem, including default commands and telemetry.
     *
     * @param subsystems The robot subsystems container.
     */
    public void setUpSwerve(Subsystems subsystems) {
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, 1.0));

        logger = new Telemetry(subsystems.swerve.getMaxSpeedMetersPerSecond(), subsystems);
        subsystems.swerve.registerTelemetry(logger::telemeterize);
    }

    /**
     * Initializes AdvantageKit logging and metadata. Configures data receivers for WPILOG and NT4.
     */
    public void setUpAdvantageScope() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
        try {
            Logger.recordMetadata(
                    "Hostname",
                    InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
        } catch (UnknownHostException e) {
            Logger.recordMetadata("Hostname", "Unknown");
        }
        Logger.recordMetadata(
                "Platform",
                "%s %s (%s)"
                        .formatted(
                                System.getProperty("os.name").replace(" ", ""),
                                System.getProperty("os.version"),
                                System.getProperty("os.arch")));
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        // Set timing mode
        setUseTiming(true);

        // Start AdvantageKit logger
        Logger.start();
    }

    /** Default constructor for the Robot class. Initializes all systems and subsystems. */
    public Robot() {
        setUpAdvantageScope();
        setUpCommandLogging();
        subsystems =
                new Subsystems(
                        new Swerve(false),
                        new LimeLightVision(),
                        new Led(90, 2),
                        new Luma(true, "object-detection"),
                        new Shooter());

        setUpSwerve(subsystems);
        publishCommandsAndSubystems(subsystems);
        setUpAutos(subsystems);
        bindDriverController();

        pose_pred = new RobotPosePredictor(subsystems.swerve);

        subsystemTriggers.SetupTriggers(subsystems, driverController, poseSupplier());

        if (isSimulation()) {
            configureFuelSim();
        }
    }

    /**
     * Constructor for the Robot class with an option to disable swerve. Useful for testing specific
     * subsystems in isolation.
     *
     * @param isSwerveDisabled True if the swerve drive should be disabled.
     */
    public Robot(boolean isSwerveDisabled) {
        setUpAdvantageScope();
        setUpCommandLogging();
        subsystems =
                new Subsystems(
                        new Swerve(isSwerveDisabled),
                        new LimeLightVision(),
                        new Led(90, 2),
                        new Luma(true, "object-detection"),
                        new Shooter());
        setUpSwerve(subsystems);
        pose_pred = new RobotPosePredictor(subsystems.swerve);
        publishCommandsAndSubystems(subsystems);
        setUpAutos(subsystems);
        bindDriverController();

        subsystemTriggers.SetupTriggers(subsystems, driverController, poseSupplier());
    }

    /**
     * Calculates the 3D pose of the turret relative to the robot's center.
     *
     * @param turretAngleDegrees The current rotation of the turret in degrees.
     * @return The Pose3d representing the turret's position and orientation.
     */
    public Pose3d getTurretPose(double turretAngleDegrees) {
        // Assuming the turret is mounted at the center of the robot and has a fixed height
        double xMeterOffset = -0.1; // X offset from robot center to turret
        double yMeterOffset = -0.12; // Y offset from robot center to turret
        double zMeterOffset = 0.3; // Height of the turret from the ground
        return new Pose3d(
                xMeterOffset,
                yMeterOffset,
                zMeterOffset,
                new Rotation3d(0, 0, turretAngleDegrees * Math.PI / 180));
    }

    /**
     * Provides a supplier for the robot's current 2D pose from the swerve drive.
     *
     * @return A supplier that returns the current Pose2d.
     */
    public Supplier<Pose2d> poseSupplier() {
        return () -> subsystems.swerve.getState().Pose;
    }

    /**
     * Calculates the 3D pose of the shooter hood relative to the robot's center.
     *
     * @param hoodAngleDegrees The current angle of the shooter hood.
     * @return The Pose3d of the hood.
     */
    public Pose3d getHoodPose(double hoodAngleDegrees) {
        double dx = 0.09; // X offset from turret center to hood
        double dy = 0.0; // Y offset from turret center to hood
        double dz = 0.12; // z offset from turret pivot to hood pivot

        Pose3d turretPose = getTurretPose(0.0);

        Pose3d hoodPosition =
                turretPose.transformBy(
                        new Transform3d(
                                dx,
                                dy,
                                dz,
                                new Rotation3d(
                                        0.0, hoodAngleDegrees * Conv.DEGREES_TO_RADIANS, 0.0)));
        return hoodPosition;
    }

    /**
     * Checks if the robot is currently positioned under a trench or bump on the field. Used for
     * driving logic and possibly avoiding collisions or adjusting vision.
     *
     * @return True if the robot is under a trench or bump.
     */
    boolean underTrench() {
        Pose2d turretPredPose = turret_pred.getPredictedPose().get().toPose2d();
        Pose2d turretAccPose = subsystems.swerve.getState().Pose;

        double dx1Pred = Math.abs(turretPredPose.getX() - FieldConstants.BUMP.BUMP_1_X_METERS);
        double dx2Pred = Math.abs(turretPredPose.getX() - FieldConstants.BUMP.BUMP_2_X_METERS);

        double dx1Acc = Math.abs(turretAccPose.getX() - FieldConstants.BUMP.BUMP_1_X_METERS);
        double dx2Acc = Math.abs(turretAccPose.getX() - FieldConstants.BUMP.BUMP_2_X_METERS);

        boolean under1Pred = dx1Pred <= .5;
        boolean under2Pred = dx2Pred <= .5;

        boolean under1Acc = dx1Acc <= .5;
        boolean under2Acc = dx2Acc <= .5;

        return under1Pred || under2Pred || under1Acc || under2Acc;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Update pose prediction
        pose_pred.setVelocitiesAndPose();

        // Update trench state
        DrivingSharedState.getInstance().setUnderTrench(underTrench());

        // Log predictor states
        turret_pred.logTurretPose(
                turret_pred.getTurretPoseFieldRelativeOffset(subsystems.swerve.getState().Pose));
        pose_pred_error.logPose(subsystems.swerve.getState().Pose);
        turret_pred_error.logPose(
                turret_pred.getTurretPoseFieldRelativeOffset(subsystems.swerve.getState().Pose));

        // Visualize poses in AdvantageScope
        Logger.recordOutput(
                "zeroedPoses",
                new Pose3d[] {
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                    new Pose3d(0, 0, 0, new Rotation3d(0, 0.0, 0))
                });

        // Integrate vision measurements
        if (kUseLimelight) {
            var driveState = subsystems.swerve.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            Pose2d currentPose =
                    subsystems.vision.getRobotPoseFromVision(headingDeg, omegaRps, 0, 0, 0, 0);

            if (currentPose != null) {
                // Trusts vision rotation less than position. Needs tuning.
                subsystems.swerve.addVisionMeasurement(
                        currentPose, subsystems.vision.getLastTimeStamp());

                if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", false);
                }
            } else {
                if (!SubsystemConstants.kLimelightVision.disableVisionLogs) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", true);
                }
            }
        }
    }

    /** Binds commands to the driver controller. */
    public void bindDriverController() {
        driverController.bind(subsystems);
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, detune.value()));

        // Sync shared state with tunables
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());

        subsystems.vision.disableCameras();
        bindDriverController();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        subsystems.vision.enableCameras(0);
    }

    @Override
    public void autonomousInit() {
        subsystems.vision.enableCameras(0);
        Command autoCommand = autoChooser.selectedCommand();
        if (fuelSim != null) {
            fuelSim.start();
        }

        m_autonomousCommand = autoCommand;
        if (autoCommand != null) {
            scheduler.schedule(autoCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        subsystems.swerve.clearActiveTrajectory();
        scheduler.cancelAll();
    }

    @Override
    public void teleopInit() {
        subsystems.vision.enableCameras(0);
        subsystems.swerve.clearActiveTrajectory();
        if (fuelSim != null) {
            fuelSim.start();
        }
        scheduler.cancelAll();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Command autoCommand = testChooser.selectedCommand();
        if (fuelSim != null) {
            fuelSim.start();
        }
        if (autoCommand != null) {
            scheduler.schedule(autoCommand);
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /**
     * Returns whether the robot is currently in test mode.
     *
     * @return True if in test mode.
     */
    public static boolean isRobotTest() {
        return RobotModeTriggers.test().getAsBoolean();
    }

    @Override
    public void simulationPeriodic() {
        if (fuelSim != null) {
            fuelSim.updateSim();

            // Example of how to integrate mechanism simulation with fuel simulation
            double currentTime = RobotController.getFPGATime() / 1.0e6;

            // TODO: Implement actual fuel launch logic tied to mechanism states
            if (false) {
                double flywheelRadius = SubsystemConstants.kShooter.kFlywheels.WHEEL_RADIUS_METERS;
                double launchVelocity = (50 * flywheelRadius) / 2.0;

                fuelSim.launchFuel(
                        MetersPerSecond.of(launchVelocity),
                        Radians.of(Math.PI / 2),
                        Degrees.of(0),
                        Meters.of(SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters));

                lastShotTime = currentTime;
                if (!SubsystemConstants.disableAllLogs) {
                    Log.log("ROBOT/Simulation/FuelLaunched", true);
                }
            }
        }
    }

    /** Configures the fuel simulation environment. */
    private void configureFuelSim() {
        fuelSim = new FuelSim();
        fuelSim.start();
        SmartDashboard.putData(
                Commands.runOnce(() -> fuelSim.clearFuel())
                        .withName("Clear Fuel")
                        .ignoringDisable(true));
        fuelSim.enableAirResistance();

        configureFuelSimRobot();
    }

    /** Registers the robot chassis and intake zones with the fuel simulation. */
    private void configureFuelSimRobot() {
        // Chassis dimensions with bumpers
        double width = 0.71;
        double length = 0.71;
        double bumperHeight = 0.2;

        fuelSim.registerRobot(
                width,
                length,
                bumperHeight,
                () -> subsystems.swerve.getState().Pose,
                subsystems.swerve::getFieldRelativeSpeeds);

        // Register a front intake zone
        fuelSim.registerIntake(
                length / 2,
                length / 2 + 0.1,
                -0.2,
                0.2,
                () -> true,
                () -> Log.log("ROBOT/Simulation/FuelIntaked", true));
    }

    /**
     * Returns whether the robot is currently on the Blue alliance. Defaults to Blue if alliance is
     * unknown.
     *
     * @return True if Blue alliance, false if Red.
     */
    public static boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            return ally.get() == Alliance.Blue;
        } else {
            if (!SubsystemConstants.disableAllLogs) {
                Log.log("ROBOT/System/AllianceUnknown", true);
            }
            return true;
        }
    }
}
