package igknighters.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.Indexer;

public class IndexerCommands {
    public static Command SummonSpindexeroidTitano(
            Indexer indexer, AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed) {
        return indexer.exitRoller
                .run(
                        () -> {
                            indexer.exitRoller.spin(exitRollerSpeed);
                        })
                .alongWith(
                        indexer.spindexer.run(
                                () -> {
                                    indexer.spindexer.spin(spindexerSpeed);
                                }));
    }

    public static Command SHUTUP(Indexer indexer) {
        return indexer.exitRoller
                .run(
                        () -> {
                            indexer.exitRoller.spin(RPM.of(0));
                        })
                .alongWith(
                        indexer.spindexer.run(
                                () -> {
                                    indexer.spindexer.spin(RPM.of(0));
                                }));
    }
}
