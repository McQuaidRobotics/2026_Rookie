package igknighters.subsystems.Fly_Wheel.hardware;

public abstract class BingusMotorAbstact {

    public abstract void setSpeed(double speedMetersPerSecond);

    public abstract void setVoltage(double voltage);

    public abstract void periodic();

    public abstract double getSpeedRPM();
}
