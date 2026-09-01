package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterFlyWheel extends SubsystemBase {
        private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(kFlywheels.kP, kFlywheels.kI, kFlywheels.kD)
                    .withSimClosedLoopController(5, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kFlywheels.MAX_SPEED_RPM/60),
                            RotationsPerSecondPerSecond.of(kFlywheels.MAX_ACCELERATION_RPM/60))
                    .withSimClosedLoopController(5, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0, 0, 0))
                    .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withGearing(1)
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withMomentOfInertia(Meters.of(.1), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withFollowers(Pair.of(new TalonFX(kFlywheels.FOLLOWER_MOTOR_ID, SubsystemConstants.superStructure), true));
                    


    private TalonFX talon = new TalonFX(kFlywheels.LEADER_MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);



    private final FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig()
                    .withTelemetry("SHOOTER_FLYWHEEL", TelemetryVerbosity.HIGH)
                    // Soft limit is applied to the SmartMotorControllers PID
                    .withTelemetry("SHOOTER_FLYWHEEL", TelemetryVerbosity.HIGH);
    private FlyWheel flyWheel = new FlyWheel(flyWheelConfig, talonSmartMotorController);
}
