package igknighters.commands;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.Indexer;

public class SpindexerCommands {
    public static Command SummonSpindexeroidTitano(
            Indexer indexer, AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed) {
        return Commands.parallel(
                indexer.spindexer.run(spindexerSpeed),
                indexer.exitRoller.setAngularVelocityCommand(exitRollerSpeed));
    }
}
