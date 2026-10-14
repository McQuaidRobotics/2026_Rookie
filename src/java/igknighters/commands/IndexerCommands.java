package igknighters.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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

    public static Command JekSpindexer(Spindexer spindexer, AngularVelocity speed) {
        return Commands.repeatingSequence(
                Commands.runOnce(
                        () -> {
                            System.out.println("Forward");
                            spindexer.setSpeedNoCommand(speed);
                        },
                        spindexer),
                Commands.waitSeconds(1),
                Commands.runOnce(
                        () -> {
                            System.out.println("reverse");
                            spindexer.setSpeedNoCommand(speed.times(-1));
                        },
                        spindexer),
                Commands.waitSeconds(1));
    }
}
