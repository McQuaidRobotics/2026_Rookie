package igknighters.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class intakePivot extends SubsystemBase {
        CANcoder turretEncoder =
            new CANcoder(
                    SubsystemConstants.kIntake.kPivot.CANCODER_ID,
                    SubsystemConstants.superStructure);

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(kIntake.kPivot.kP, kIntake.kPivot.kI, kIntake.kPivot.kD)
                    .withSimClosedLoopController(kIntake.kPivot.kP, kIntake.kPivot.kI, kIntake.kPivot.kD)
                    .withTrapezoidalProfile(
                            MetersPerSecond.of(kIntake.kPivot.MAX_SPEED_METERS_PER_SECOND),
                            MetersPerSecondPerSecond.of(kIntake.kPivot.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED / 60))
                    // ----------------------------------------------------------------
                    .withSoftLimits(
                            Degrees.of(kIntake.kPivot.MIN_ANGLE_DEGREES),
                            Degrees.of(kIntake.kPivot.MAX_ANGLE_DEGREES))
                    // ----------------------------------------------------------------
                    .withSimClosedLoopController(5, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(kIntake.kPivot.kS, 0, kIntake.kPivot.kV))
                    .withSimFeedforward(
                            new ArmFeedforward(
                                    kIntake.kPivot.kS,
                                    0,
                                    kIntake.kPivot.kV)) // kg is not nesessary because the turret is
                    // horizontal and does not have to fight gravity
                    // Telemetry name and verbosity level
                    .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
                    .withGearing(kIntake.kPivot.GEAR_RATIO)
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(kIntake.kPivot.STATOR_CURRENT_LIMIT))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimits(
                            Degrees.of(kIntake.kPivot.MIN_ANGLE_DEGREES),
                            Degrees.of(kIntake.kPivot.MAX_ANGLE_DEGREES))
                    .withMomentOfInertia(Meters.of(kIntake.kPivot.LENGTH_METERS), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(
                            Seconds.of(0.25)) //  numbers that we have seen work in the season
                    .withExternalEncoder(turretEncoder)
                    .withExternalEncoderInverted(false)
                    .withExternalEncoderGearing(
                            new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withExternalEncoderZeroOffset(
                            Rotations.of(
                                    kIntake.kPivot.ENCODER_OFFSET)) // this is what allows you
                    // to zero the encoder
                    .withUseExternalFeedbackEncoder(true)
                    .withStartingPosition(Degrees.of(kIntake.kPivot.MIN_ANGLE_DEGREES));

    private TalonFX talon = new TalonFX(kIntake.kPivot.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getFalcon500(1), smcConfig);

    private final PivotConfig pivotConfig =
            new PivotConfig()
                    // Soft limit is applied to the SmartMotorControllers PID

                    .withHardLimits(Degrees.of(-280), Degrees.of(100))
                    .withTelemetry("PIVOT", TelemetryVerbosity.HIGH);
    private Pivot pivot = new Pivot(pivotConfig, talonSmartMotorController);


    public void targetAngle(Angle angle) {
        // shooter.setMeasurementPositionSetpoint(); OG
        pivot.setMechanismPositionSetpoint(angle); // fix
    }

    public Angle getCurrentAngle() {
        return pivot.getAngle();
    }

    public Command targetAngleCommand(Angle angle) {
        return pivot.run(angle);
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        return pivot.runTo(angle, Rotations.of(.05));
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
    

