package igknighters.subsystems.Fly_Wheel.hardware;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BingusMotor extends SubsystemBase {
    BingusMotorAbstact motor;

    public BingusMotor() {
        motor = new BingusMotorFR();
    }

    public Command targetSpeed(double RPM) {
        return this.run(() -> motor.setSpeed(RPM));
    }
}
