package igknighters.subsystems.Intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeState {
    public Angle pivotAngle;
    public AngularVelocity rollerSpeed;

    public IntakeState(Angle pivotAngle, AngularVelocity rollerSpeed) {
        this.pivotAngle = pivotAngle;
        this.rollerSpeed = rollerSpeed;
    }
}
