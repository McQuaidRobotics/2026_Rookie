package igknighters.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public final ExitRoller exitRoller = new ExitRoller();
    public final Spindexer spindexer = new Spindexer();
}
