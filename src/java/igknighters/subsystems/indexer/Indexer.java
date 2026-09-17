package igknighters.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public final ExitRoller exitRoller = new ExitRoller();
    public final Spindexer spindexer = new Spindexer();

    public Command runExitRoller(AngularVelocity ExitRollerSpeed) {
        return exitRoller.setAngularVelocityCommand(ExitRollerSpeed);
    }

    public Command runSpindexer(AngularVelocity SpindexerSpeed) {
        return spindexer.run(SpindexerSpeed);
    }

    public Command setExitRollerVoltage(Voltage voltage) {
        return exitRoller.setVoltageCommand(voltage);
    }

    public Command setSpindexerVoltage(Voltage voltage) {
        return spindexer.setVoltage(voltage);
    }
}
