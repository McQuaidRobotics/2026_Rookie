package igknighters.subsystems.SPINNY;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Spinny extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(1, 0, 0)
                    .withSimClosedLoopController(1, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
                    // Telemetry name and verbosity level
                    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                    // Gearing from the motor rotor to final shaft.
                    // In this example GearBox.fromReductionStages(3,4) is the same as
                    // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                    // your motor.
                    // You could also use .withGearing(12) which does the same thing.
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.COAST)
                    .withStatorCurrentLimit(Amps.of(40));

    // Vendor motor controller object
    private TalonFX talon = new TalonFX(14); // anything other then 1-12 15 or 16

    // Create our SmartMotorController from our TalonFX and config.
    private SmartMotorController smc =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    private final FlyWheelConfig shooterConfig =
            new FlyWheelConfig()
                    // Diameter of the flywheel.
                    .withDiameter(Inches.of(4))
                    .withSmartMotorController(smc)
                    .withMass(Pounds.of(1))
                    // Telemetry name and verbosity for the arm.
                    .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    private FlyWheel shooter = new FlyWheel(shooterConfig);

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }

    /**
     * Set the shooter velocity setpoint.
     *
     * @param speed Speed to set
     */
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }

    /**
     * Runs the shooter at the given velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command run(AngularVelocity speed) {
        return shooter.run(speed);
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        shooter.simIterate();
    }
}
