package igknighters.subsystems.intake.roller;

public class RollerDisabled extends Roller {
    @Override
    public void setSpeed(double speed){
        //does nothing
    }

    @Override
    public double getPosition(){
        return 0;
    }

    @Override
    public void stop(){
        //does nothing
    };
}
