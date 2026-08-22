package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.commands.shooter.ShooterCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.turret.TurretNoAbstract;
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

    public final TurretNoAbstract turret;

    public final Shooter shooter;

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
    public Subsystems(Swerve swerve, LimeLightVision vision, Led led, Luma luma, Shooter shooter) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.luma = luma;
        this.shooter = shooter;
        this.lockedResources = new SubsystemBase[] {swerve, vision, led, luma};
        this.turret = shooter.turret;

        this.turret.setDefaultCommand(ShooterCommands.faceForward(shooter));
    }
}
