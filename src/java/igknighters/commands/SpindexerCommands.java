package igknighters.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.indexer.Indexer;

public class SpindexerCommands {
    public static Command SummonSpindexeroidTitano(
            Indexer indexer, AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed) {
        return indexer.exitRoller
                .run(exitRollerSpeed)
                .alongWith(indexer.spindexer.run(spindexerSpeed));
    }

    public static Command SHUTUP(Indexer spindexer) {
        return spindexer.run(
                () -> {
                    spindexer.setExitRollerVoltage(Volts.of(0));
                    spindexer.setSpindexerVoltage(Volts.of(0));
                });
    }
}
