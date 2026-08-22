package igknighters.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import igknighters.constants.SubsystemConstants.kShooter.kTurret;
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

public class TurretNoAbstract extends SubsystemBase {

    CANcoder turretEncoder =
            new CANcoder(
                    SubsystemConstants.kShooter.kTurret.CANCODER_ID,
                    SubsystemConstants.superStructure);

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(kTurret.kP, kTurret.kI, kTurret.kD)
                    .withSimClosedLoopController(kTurret.kP, kTurret.kI, kTurret.kD)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kTurret.MAX_SPEED_RPM), RotationsPerSecondPerSecond.of(kTurret.MAX_ACCELERATION_RPM))
                    // ----------------------------------------------------------------
                    .withSoftLimits(Degrees.of(kTurret.MIN_ANGLE_DEGREES), Degrees.of(kTurret.MAX_ANGLE_DEGREES))
                    // ----------------------------------------------------------------
                    .withSimClosedLoopController(50, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(kTurret.kS, 0, kTurret.kV))
                    .withSimFeedforward(new ArmFeedforward(kTurret.kS, 0, kTurret.kV)) // kg is not nesessary because the turret is horizontal and does not have to fight gravity
                    // Telemetry name and verbosity level
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)

                    .withGearing(16.2)
                    // Motor properties to prevent over currenting.
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimits(Degrees.of(kTurret.MIN_ANGLE_DEGREES), Degrees.of(kTurret.MAX_ANGLE_DEGREES))
                    .withMomentOfInertia(Meters.of(.1), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25)) //  numbers that we have seen work in the season
                    .withExternalEncoder(turretEncoder) 
                    .withExternalEncoderInverted(false)
                    .withExternalEncoderGearing(
                            new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withExternalEncoderZeroOffset(
                            Rotations.of(
                                    kTurret.CANCODER_OFFSET_ROTATIONS)) // this is what allows you
                    // to zero the encoder
                    .withUseExternalFeedbackEncoder(true)
                    .withStartingPosition(Degrees.of(0));

    private TalonFX talon = new TalonFX(kTurret.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getFalcon500(1), smcConfig);

    private final PivotConfig shooterConfig =
            new PivotConfig()
                    // Soft limit is applied to the SmartMotorControllers PID

                    .withHardLimits(Degrees.of(-280), Degrees.of(100))
                    .withTelemetry("SHOOTER_TURRET", TelemetryVerbosity.HIGH);
    private Pivot shooter = new Pivot(shooterConfig, talonSmartMotorController);

        public static Angle wrapAngle(Angle angle) {
        double ogDegrees = angle.in(Degrees);
        double maxDegrees = SubsystemConstants.kShooter.kTurret.MAX_ANGLE_DEGREES;
        double MIN_ANGLE_DEGREES = SubsystemConstants.kShooter.kTurret.MIN_ANGLE_DEGREES;
        double width = maxDegrees - MIN_ANGLE_DEGREES;

        double newDegs =
                MIN_ANGLE_DEGREES + (((ogDegrees - MIN_ANGLE_DEGREES) % width + width) % width);

        return Degrees.of(newDegs);
    }

    public void targetAngle(Angle angle) {
        // shooter.setMeasurementPositionSetpoint(); OG
        shooter.setMechanismPositionSetpoint(wrapAngle(angle)); // fix
    }

    public Angle getCurrentAngle() {
        return shooter.getAngle();
    }

    public Command targetAngleCommand(Angle angle) {
        return shooter.run(angle);
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        return shooter.runTo(angle, Rotations.of(.05));
    }

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }
}
