package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer.kExitRollers;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ExitRoller extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(kExitRollers.kP, kExitRollers.kI, kExitRollers.kD)
                    .withSimClosedLoopController(kExitRollers.kP, kExitRollers.kI, kExitRollers.kD)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kExitRollers.MAX_SPEED_RPM / 60),
                            RotationsPerSecondPerSecond.of(kExitRollers.MAX_ACCELERATION_RPM / 60))
                    // Feedforward Constants
                    .withTelemetry("ExitRollerMotor", TelemetryVerbosity.HIGH)
                    .withGearing(1)
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(kExitRollers.STATOR_CURRENT_LIMIT))
                    .withMomentOfInertia(
                            KilogramSquareMeters.of(kExitRollers.MOMENT_OF_INERTIA_KG_M2));

    private TalonFX talon =
            new TalonFX(
                    kExitRollers.LEADER_MOTOR_ID,
                    SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX44(1), smcConfig);

    private final FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig().withTelemetry("ExitRollerMech", TelemetryVerbosity.HIGH);
    private FlyWheel flyWheel = new FlyWheel(flyWheelConfig, talonSmartMotorController);

    public Command setSpeed(AngularVelocity speed) {
        return this.run(() -> flyWheel.setMechanismVelocitySetpoint(speed));
    }

    public Command setVoltage(Voltage voltage) {
        return flyWheel.setVoltage(voltage);
    }

    public void setSpeedNoCommand(AngularVelocity speed) {
        flyWheel.setMechanismVelocitySetpoint(speed);
    }

    public void setVoltageNoCommand(Voltage voltage) {
        flyWheel.setVoltageSetpoint(voltage);
    }

    @Override
    public void simulationPeriodic() {
        flyWheel.simIterate();
    }

    @Override
    public void periodic() {
        flyWheel.updateTelemetry();
    }
}
