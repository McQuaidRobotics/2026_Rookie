package igknighters.subsystems.Fly_Wheel.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;

public class BingusMotorFR extends BingusMotorAbstact {
    private final TalonFX mainShooter = new TalonFX(4);
    //     private final TalonFX followerShooter =
    //             new TalonFX(SubsystemConstants.kShooter.kRollers.FOLLOWER_MOTOR_ID);

    private final MotionMagicVelocityVoltage velocityControl;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    private BaseStatusSignal shooterVelocity;
    private BaseStatusSignal shooterCurrent;
    private BaseStatusSignal shooterVoltage;
    private BaseStatusSignal shooterTemperature;

    public TalonFXConfiguration getLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;

        config.Feedback.SensorToMechanismRatio = 1;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // config.MotionMagic.MotionMagicJerk =
        // SubsystemConstants.kShooter.kRollers.MOTION_MAGIC_JERK;
        // config.MotionMagic.MotionMagicAcceleration =
        //         SubsystemConstants.kShooter.kRollers.MAX_ACCELERATION_RPM;
        // config.MotionMagic.MotionMagicCruiseVelocity =
        //         SubsystemConstants.kShooter.kRollers.MAX_SPEED_RPM;
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.SupplyCurrentLimit =
        //         SubsystemConstants.kShooter.kRollers.SUPPLY_CURRENT_LIMIT;
        // config.MotorOutput.PeakReverseDutyCycle = 0.0;
        // config.TorqueCurrent.PeakForwardTorqueCurrent =
        //         SubsystemConstants.kShooter.kRollers.PEAK_CURRENT_LIMIT;

        return config;
    }

    public BingusMotorFR() {

        mainShooter.getConfigurator().apply(getLeaderConfig());
        // followerShooter.setControl(
        //         new Follower(mainShooter.getDeviceID(), MotorAlignmentValue.Aligned));

        velocityControl = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        shooterVelocity = mainShooter.getVelocity();
        shooterCurrent = mainShooter.getSupplyCurrent();
        shooterVoltage = mainShooter.getSupplyVoltage();
        shooterTemperature = mainShooter.getDeviceTemp();
    }

    @Override
    public void setSpeed(double speedRpm) {
        DogLog.log("Subsystems/Shooter/Rollers/setSpeed", speedRpm);
        mainShooter.setControl(velocityControl.withVelocity(speedRpm / 60.0));
    }

    @Override
    public void setVoltage(double voltage) {
        mainShooter.setControl(dutyCycleControl.withOutput(voltage / 12.0));
    }

    @Override
    public double getSpeedRPM() {
        return shooterVelocity.getValueAsDouble();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                shooterVelocity, shooterCurrent, shooterVoltage, shooterTemperature);
        DogLog.log(
                "Subsystems/Shooter/Rollers/velocity", shooterVelocity.getValueAsDouble() * 60.0);
        DogLog.log("Subsystems/Shooter/Rollers/current", shooterCurrent.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/voltage", shooterVoltage.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/temperature", shooterTemperature.getValueAsDouble());
        DogLog.log("Subsystems/Shooter/Rollers/periodicing", true);
    }
}
