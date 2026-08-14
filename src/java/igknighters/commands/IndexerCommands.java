package igknighters.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.Indexer;

public class IndexerCommands {
    public static Command SummonSpindexeroidTitano(
            Indexer indexer, AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed) {
        Command command =
                indexer.exitRoller
                        .run(exitRollerSpeed)
                        .alongWith(indexer.spindexer.run(spindexerSpeed));
        command.addRequirements(indexer);
        return command;
    }

    public static Command SHUTUP(Indexer indexer) {
        Command command =
                indexer.exitRoller.run(RPM.of(0)).alongWith(indexer.spindexer.run(RPM.of(0)));
        command.addRequirements(indexer);
        return command;
    }

    public static Command defaultCMD(Indexer indexer) {
        indexer.run()
    }
}
