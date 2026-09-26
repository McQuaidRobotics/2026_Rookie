package igknighters.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public final ExitRoller exitRoller = new ExitRoller();
    public final Spindexer spindexer = new Spindexer();

    public Command setState(AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed){
        return Commands.parallel(exitRoller.setSpeed(exitRollerSpeed), spindexer.setSpeed(spindexerSpeed));
    }

    public void setStateNoCommand(AngularVelocity exitRollerSpeed, AngularVelocity spindexerSpeed){
        exitRoller.setSpeedNoCommand(exitRollerSpeed);
        spindexer.setSpeedNoCommand(spindexerSpeed);
    }
}
