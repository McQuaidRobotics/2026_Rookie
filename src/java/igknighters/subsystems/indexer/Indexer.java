package igknighters.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public final ExitRoller exitRoller = new ExitRoller();
    public final Spindexer spindexer = new Spindexer();

    public void runExitRoller(AngularVelocity ExitRollerSpeed) {
        exitRoller.run(ExitRollerSpeed);
    }

    public void runSpindexer(AngularVelocity SpindexerSpeed) {
        spindexer.run(SpindexerSpeed);
    }

    public void setExitRollerVoltage(Voltage voltage) {
        exitRoller.setVoltage(voltage);
    }

    public void setSpindexerVoltage(Voltage voltage) {
        spindexer.setVoltage(voltage);
    }
}
