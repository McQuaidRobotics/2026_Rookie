package igknighters.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.Spindexer;

public class IndexerCommands {
    public static Command SummonSpindexeroidTitano(
            Indexer indexer, AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed) {
        return Commands.parallel(
                indexer.spindexer.setSpeed(spindexerSpeed),
                indexer.exitRoller.setSpeed(exitRollerSpeed));
    }
    public static Command JekSpindexer(Spindexer spindexer, AngularVelocity speed){
        return Commands.repeatingSequence(
            spindexer.setSpeed(speed),
            Commands.waitSeconds(1),
            spindexer.setSpeed(speed.times(-1)),
            Commands.waitSeconds(1)
        );
    }
}
