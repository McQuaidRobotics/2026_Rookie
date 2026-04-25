package igknighters.subsystems.intake.roller;

public class RollerSim extends Roller {
    private double currentSpeed = 0.0 ;
    private double currentDistance = 0.0;


    public RollerSim(){

    }

    @Override
    public void setSpeed(double speed){
        currentSpeed = speed;
    }

    /*
     * simulation update
     * @return distance in meters
     */
    @Override
    public double getPosition(){
        currentDistance += currentDistance * 0.2;
        return currentDistance;
    }

    @Override
    public void stop(){
        setSpeed(0.0);
    }
}
