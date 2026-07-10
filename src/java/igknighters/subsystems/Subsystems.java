package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.commands.SpindexerCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.Swerve;

/**
 * Central container for all robot subsystems. This class facilitates easy access to subsystems from
 * the Robot class and during command binding.
 */
public class Subsystems {
    /** The swerve drive subsystem. */
    public final Swerve swerve;

    /** The vision subsystem (Limelight). */
    public final LimeLightVision vision;

    /** The LED subsystem for visual feedback. */
    public final Led led;

    /** The Luma subsystem for object detection. */
    public final Luma luma;

    public final Indexer spindexer;

    /**
     * Array of subsystems that require exclusive access (Locked resources). Used for publishing
     * command data and managing command requirements.
     */
    public final SubsystemBase[] lockedResources;

    /**
     * Constructs the Subsystems container with the provided instances.
     *
     * @param swerve The swerve drive subsystem.
     * @param vision The vision subsystem.
     * @param led The LED subsystem.
     * @param luma The Luma subsystem.
     */
    public Subsystems(
            Swerve swerve, LimeLightVision vision, Led led, Luma luma, Indexer spindexer) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.luma = luma;
        this.spindexer = spindexer;
        this.lockedResources = new SubsystemBase[] {swerve, vision, led, luma};

        this.spindexer.setDefaultCommand(SpindexerCommands.SHUTUP(this.spindexer));
    }
}
