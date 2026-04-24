package igknighters.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelReal extends Flywheel {
    private final TalonFXConfiguration talonFXConfigs;

    public FlywheelReal() {
        this.talonFXConfigs = new TalonFXConfiguration();
    }



    @Override
    public AngularVelocity getSpeed() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setSpeed(AngularVelocity speed) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        
    }

}