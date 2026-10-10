package igknighters.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kIntake;
import igknighters.util.log.Log;
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

public class IntakePivot extends SubsystemBase {
    CANcoder pivotIncoder =
            new CANcoder(
                    SubsystemConstants.kIntake.kPivot.CANCODER_ID,
                    SubsystemConstants.superStructure);

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(
                            kIntake.kPivot.kP, kIntake.kPivot.kI, kIntake.kPivot.kD)
                    .withSimClosedLoopController(5, 0, 0)
                    //     .withTrapezoidalProfile(
                    //             MetersPerSecond.of(kIntake.kPivot.MAX_SPEED_METERS_PER_SECOND),
                    //             MetersPerSecondPerSecond.of(
                    //                     kIntake.kPivot.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                    // / 60))
                    // ----------------------------------------------------------------
                    .withSoftLimits(
                            Degrees.of(kIntake.kPivot.MIN_ANGLE_DEGREES - 3),
                            Degrees.of(
                                    kIntake.kPivot.MAX_ANGLE_DEGREES
                                            + 3)) // soft limits are applied to the
                    // SmartMotorControllers PID
                    // ----------------------------------------------------------------
                    // Feedforward Constants
                    .withSimFeedforward(
                            new ArmFeedforward(
                                    kIntake.kPivot.kS,
                                    0,
                                    kIntake.kPivot.kV)) // kg is not nesessary because the turret is
                    // horizontal and does not have to fight gravity
                    // Telemetry name and verbosity level
                    .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
                    .withGearing(1) // gearing is 1:1 because the gearing is already applied in the
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(true)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(kIntake.kPivot.STATOR_CURRENT_LIMIT))
                    //     .withClosedLoopRampRate(Seconds.of(0.25))
                    .withMomentOfInertia(Meters.of(kIntake.kPivot.LENGTH_METERS), Pounds.of(.15))
                    //     .withOpenLoopRampRate(
                    //             Seconds.of(0.25)) //  numbers that we have seen work in the
                    // season
                    .withExternalEncoder(pivotIncoder)
                    .withExternalEncoderInverted(true)
                    .withExternalEncoderGearing(
                            new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withExternalEncoderZeroOffset(
                            Rotations.of(kIntake.kPivot.ENCODER_OFFSET)) // this is what allows you
                    // to zero the encoder
                    .withUseExternalFeedbackEncoder(true)
                    .withStartingPosition(Degrees.of(kIntake.kPivot.MIN_ANGLE_DEGREES));

    private TalonFX talon = new TalonFX(kIntake.kPivot.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

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
        Log.log("ROBOT/SUBSYSTEMS/INTAKE/PIVOT/TARGETING_POSITION", angle.in(Degrees));
        return pivot.run(angle);
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        Log.log("ROBOT/SUBSYSTEMS/INTAKE/PIVOT/TARGETING_POSITION", angle.in(Degrees));
        return pivot.runTo(angle, Rotations.of(.05));
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        Log.log("ROBOT/SUBSYSTEMS/INTAKE/PIVOT/CANCODER_POSITION", pivot.getAngle().in(Degrees));
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
