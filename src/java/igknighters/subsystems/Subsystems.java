package igknighters.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
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

    public final Shooter shooter;

    public final Indexer indexer;

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
     * @param shooter The shooter subsystem.
     */
    public Subsystems(
            Swerve swerve,
            LimeLightVision vision,
            Led led,
            Luma luma,
            Shooter shooter,
            Indexer indexer) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.vision = vision;
        this.led = led;
        this.luma = luma;
        this.indexer = indexer;
        this.lockedResources =
                new SubsystemBase[] {
                    swerve, vision, led, luma, shooter, shooter.hood, shooter.turret, shooter.flyWheel, indexer, indexer.spindexer, indexer.exitRoller
                };
        this.shooter.hood.setDefaultCommand(
                shooter.hood.targetAngleCommand(Degrees.of(kHood.MIN_ANGLE_DEGREES)));
        this.shooter.turret.setDefaultCommand(
                this.shooter.turret.targetAngleCommand(Degrees.of(0)));
        this.indexer.spindexer.setDefaultCommand(this.indexer.spindexer.setVoltage(Volts.of(0)));
        this.indexer.exitRoller.setDefaultCommand(this.indexer.exitRoller.setVoltage(Volts.of(0)));
        this.shooter.flyWheel.setDefaultCommand(
                this.shooter
                        .flyWheel
                        .setSpeed(RPM.of(2000))
                        .withName("IDLING THE FLYWHEELS AT 2000 RPM"));
    }
}
