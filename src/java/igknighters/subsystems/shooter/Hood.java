package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.log.Log;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class Hood extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withStartingPosition(Degrees.of(kHood.MIN_ANGLE_DEGREES))
                    // Feedback Constants (PID Constants)
                    .withClosedLoopController(kHood.kP, kHood.kI, kHood.kD)
                    .withSimClosedLoopController(5, 0, 0)
                    .withTrapezoidalProfile(
                            RotationsPerSecond.of(kHood.MAX_SPEED_R_P_S),
                            RotationsPerSecondPerSecond.of(kHood.MAX_ACCEL_R_P_S_S))
                    // ----------------------------------------------------------------
                    .withSoftLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    // ----------------------------------------------------------------
                    .withSimClosedLoopController(5, 0, 0)
                    // Feedforward Constants
                    .withFeedforward(new ArmFeedforward(0, 0, 0))
                    .withSimFeedforward(new ArmFeedforward(0, 0, 0))
     
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                
                    .withGearing(360 / 15)
                    .withMotorInverted(false)
                    .withIdleMode(MotorMode.BRAKE)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withMomentOfInertia(Meters.of(.1), Pounds.of(.15))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25));

    private DigitalInput dio = new DigitalInput(kHood.REVERSE_LIMIT_SWITCH_ID); 

    private TalonFX talon = new TalonFX(kHood.MOTOR_ID, SubsystemConstants.superStructure);

    private SmartMotorController talonSmartMotorController =
            new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    MechanismPositionConfig hoodPosConfig =
            new MechanismPositionConfig()
                    .withRelativePosition(
                            new Translation3d(0.2, 0.0, 0.3)) 
                    .withMaxRobotLength(Meters.of(0.85))
                    .withMaxRobotHeight(Meters.of(1.20))
                    .withMovementPlane(MechanismPositionConfig.Plane.XZ);

    private final PivotConfig hoodConfig =
            new PivotConfig()
                    .withHardLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withMechanismPositionConfig(hoodPosConfig)
                    .withTelemetry("PivotExample", TelemetryVerbosity.HIGH)
                    // Soft limit is applied to the SmartMotorControllers PID
                    .withHardLimits(
                            Degrees.of(kHood.MIN_ANGLE_DEGREES),
                            Degrees.of(kHood.MAX_ANGLE_DEGREES))
                    .withTelemetry("SHOOTER_HOOD", TelemetryVerbosity.HIGH);
    private Pivot hood = new Pivot(hoodConfig, talonSmartMotorController);

    private final Sensor hoodLimit =
            new SensorConfig("hoodLimit") // Name of the sensor
                    .withField(
                            "Limit", dio::get,
                            false) // Add a Field to the sensor named "Limit" 
                    .withSimulatedValue(
                            "Limit",
                            hood.isNear(Degrees.of(kHood.MIN_ANGLE_DEGREES), Degrees.of(2)),
                            true) 
                    .getSensor(); // Get the sensor.

                    // This is a simulated and real sensor that will return true when the hood is near the minimum angle.

    public boolean getHoodLimit() {
        Log.log("ROBOT/SUBSYSTEMS/SHOOTER/HOOD/LIMIT_HIT", hoodLimit.getAsBoolean("Limit"));
        Log.log(
                "ROBOT/SUBSYSTEMS/SHOOTER/HOOD/POSITION_ERROR",
                hood.getAngle().minus(Degrees.of(kHood.MIN_ANGLE_DEGREES)).in(Degrees));
        return hoodLimit.getAsBoolean("Limit");
    }

    public void setHoodVoltage(double voltage) {
        talonSmartMotorController.setVoltage(Volts.of(voltage));
    }

    public Angle getCurrentAngle() {
        return hood.getAngle();
    }

    public Command targetAngleCommand(Angle angle) {
        return this.run(() -> hood.setMechanismPositionSetpoint(angle));
    }

    public Command targetAngleAndEndWhenReached(Angle angle) {
        return hood.runTo(angle, Rotations.of(.05));
    }

    public static boolean isZeroed = false;

    public void zeroAt(Angle angle) {
        talonSmartMotorController.setPosition(angle);
    }

    public void targetAngleNoCommand(Angle angle) {
        Log.log("ROBOT/Subsystems/Shooter/Hood/Target_Position", angle.in(Degrees));
        hood.setMechanismPositionSetpoint(angle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
        Log.log("ROBOT/SUBSYSTEMS/SHOOTER/HOOD/POSITION", hood.getAngle().in(Degrees));
        Log.log("ROBOT/SUBSYSTEMS/SHOOTER/HOOD/LIMIT_NO_YAM", dio.get());
        if (getHoodLimit() == false) {
            isZeroed = false;
        }
        if (getHoodLimit() == true && isZeroed == false) {
            talon.setPosition(Degrees.of(kHood.MIN_ANGLE_DEGREES));
            isZeroed = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
