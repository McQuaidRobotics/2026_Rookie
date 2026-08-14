package igknighters.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer {
    public final ExitRoller exitRoller = new ExitRoller();
    public final Spindexer spindexer = new Spindexer();

    public Command runExitRoller(AngularVelocity ExitRollerSpeed) {
        System.out.println("Exit Roller Running");
        return exitRoller.run(ExitRollerSpeed);
    }

    public Command runSpindexer(AngularVelocity SpindexerSpeed) {
        return spindexer.run(SpindexerSpeed);
    }

    public Command targetState(AngularVelocity spiny, AngularVelocity exity) {
        return Commands.run( 
            spindexer.run(spiny),
            exitRoller.run(exity)
        ).addRequirements(exitRoller, spindexer);
    }

    public Command setExitRollerVoltage(Voltage voltage) {
        return exitRoller.setVoltage(voltage);
    }

    public Command setSpindexerVoltage(Voltage voltage) {
        return spindexer.setVoltage(voltage);
    }
}
