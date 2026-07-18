package igknighters.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIndexer.kSpindexer;
import igknighters.util.log.Log;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Spindexer extends SubsystemBase {

    private CANcoder spindexerEncoder =
            new CANcoder(
                    SubsystemConstants.kShooter.kTurret.CANCODER_ID,
                    new CANBus("SuperStructureBus"));
    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(1, 0, 0)
                    .withSimClosedLoopController(1, 0, 0)
                    .withFeedforward(new SimpleMotorFeedforward(0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0))
                    .withTelemetry("SpinnerMotor", TelemetryVerbosity.HIGH)
                    .withGearing(new MechanismGearing(1))
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withExternalEncoder(spindexerEncoder)
                    .withExternalEncoderInverted(false);

    private TalonFX kraken = new TalonFX(kSpindexer.LEADER_MOTOR_ID);

    private SmartMotorController smc =
            new TalonFXWrapper(kraken, DCMotor.getKrakenX44(1), smcConfig);

    private FlyWheelConfig flyWheelConfig =
            new FlyWheelConfig()
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withTelemetry("SpinnerMech", TelemetryVerbosity.HIGH)
                    .withSmartMotorController(smc);

    private FlyWheel spinner = new FlyWheel(flyWheelConfig);

    public AngularVelocity getVelocity() {
        return spinner.getSpeed();
    }

    public Command setVoltage(Voltage voltage) {
        return spinner.setVoltage(voltage);
    }

    public Command run(AngularVelocity speed) {
        Log.log("ROBOT/Commands/Indexer/Spindexer/RUNNING_AT:", speed.in(RPM));
        return spinner.run(speed);
    }

    @Override
    public void periodic() { // called in Spindexer.java
        spinner.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() { // called in Spindexer.java
        spinner.simIterate();
    }
}
