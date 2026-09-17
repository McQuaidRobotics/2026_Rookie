package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants.kIndexer.kExitRollers;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ExitRoller extends SubsystemBase {
    private CANcoder exitRollerEncoder = new CANcoder(17, new CANBus("SuperStructureBus"));

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(1, 0, 0)
                    .withSimClosedLoopController(1, 0, 0)
                    .withFeedforward(new SimpleMotorFeedforward(0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0))
                    .withTelemetry("ExitRollerMotor", TelemetryVerbosity.HIGH)
                    .withGearing(new MechanismGearing(1))
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withExternalEncoder(exitRollerEncoder)
                    .withExternalEncoderInverted(false);

    private TalonFX kraken = new TalonFX(kExitRollers.LEADER_MOTOR_ID);

    private SmartMotorController smc =
            new TalonFXWrapper(kraken, DCMotor.getKrakenX44(1), smcConfig);

    private FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig()
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withTelemetry("ExitRollerMech", TelemetryVerbosity.HIGH)
                    .withSmartMotorController(smc);

    private FlyWheel exitRoller = new FlyWheel(flyWheelConfig);

    public AngularVelocity getVelocity() {
        return exitRoller.getSpeed();
    }

    public Command setVoltageCommand(Voltage voltage) {
        return exitRoller.setVoltage(voltage);
    }

    public Command setAngularVelocityCommand(AngularVelocity speed) {
        return exitRoller.run(speed);
    }

    @Override
    public void periodic() {
        exitRoller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        exitRoller.simIterate();
    }
}
