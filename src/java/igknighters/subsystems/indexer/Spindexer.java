package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Spindexer extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(kFlywheels.kP, kFlywheels.kI, kFlywheels.kD)
                    .withSimClosedLoopController(50, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kFlywheels.MAX_SPEED_RPM / 60),
                            RotationsPerSecondPerSecond.of(kFlywheels.MAX_ACCELERATION_RPM / 60))
                    // Feedforward Constants
                    .withTelemetry("SpindexerMotor", TelemetryVerbosity.HIGH)
                    .withGearing(1)
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withMomentOfInertia(Meters.of(.05), Pounds.of(.5));

    private TalonFX talon =
            new TalonFX(18, SubsystemConstants.superStructure); // kyle you can change the id maybe.

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX44(1), smcConfig);

    private final FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig().withTelemetry("SpindexerMech", TelemetryVerbosity.HIGH);
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

    @Override
    public void simulationPeriodic() {
        flyWheel.simIterate();
    }

    @Override
    public void periodic() {
        flyWheel.updateTelemetry();
    }
}
