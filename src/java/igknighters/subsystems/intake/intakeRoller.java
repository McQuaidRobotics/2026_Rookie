package igknighters.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake.kRollers;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class intakeRoller extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(kRollers.kP, kRollers.kI, kRollers.kD)
                    .withSimClosedLoopController(50, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kRollers.MAX_SPEED_RPM / 60),
                            RotationsPerSecondPerSecond.of(kRollers.MAX_ACCELERATION_RPM / 60))
                    // Feedforward Constants
                    .withTelemetry("ROLLERS_LEADER_MOTOR", TelemetryVerbosity.HIGH)
                    .withGearing(1)
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(kRollers.STATOR_CURRENT_LIMIT))
                    .withMomentOfInertia(Meters.of(.05), Pounds.of(.5))
                    .withFollowers(
                            Pair.of(
                                    new TalonFX(
                                            kRollers.FOLLOWER_MOTOR_ID,
                                            SubsystemConstants.superStructure),
                                    true));

    private TalonFX talon =
            new TalonFX(kRollers.LEADER_MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig rollerConfig =
            new FlyWheelConfig().withTelemetry("ROLLERS_LEADER_MOTOR", TelemetryVerbosity.HIGH);
    private FlyWheel roller = new FlyWheel(rollerConfig, talonSmartMotorController);

    public Command setSpeed(AngularVelocity speed) {
        return this.run(() -> roller.setMechanismVelocitySetpoint(speed));
    }

    public void setSpeedNoCommand(AngularVelocity speed) {
        roller.setMechanismVelocitySetpoint(speed);
    }

    public AngularVelocity getSpeed() {
        return roller.getSpeed();
    }

    @Override
    public void simulationPeriodic() {
        roller.simIterate();
    }

    @Override
    public void periodic() {
        roller.updateTelemetry();
    }
}
