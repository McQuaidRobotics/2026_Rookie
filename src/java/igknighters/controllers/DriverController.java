package igknighters.controllers;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.Wayfinder;
import igknighters.subsystems.Subsystems;
import java.util.function.DoubleSupplier;

/**
 * Wrapper for the driver's Xbox controller, providing a clean interface for button triggers and
 * stick axes.
 *
 * <p>This class abstracts the raw {@link CommandXboxController} and provides named {@link Trigger}
 * objects for all buttons and D-pad directions. It is also the primary location for binding robot
 * commands to driver inputs.
 */
public class DriverController {

    private final CommandXboxController controller;

    /** Button: A (Green) */
    protected final Trigger A;

    /** Button: B (Red) */
    protected final Trigger B;

    /** Button: X (Blue) */
    protected final Trigger X;

    /** Button: Y (Yellow) */
    protected final Trigger Y;

    /** Button: Back (Left Center) */
    protected final Trigger Back;

    /** Button: Start (Right Center) */
    protected final Trigger Start;

    /** Button: Left Bumper */
    protected final Trigger LB;

    /** Button: Right Bumper */
    protected final Trigger RB;

    /** Button: Left Stick Click */
    protected final Trigger LS;

    /** Button: Right Stick Click */
    protected final Trigger RS;

    /** Trigger: Left (Pressure sensitive) */
    protected final Trigger LT;

    /** Trigger: Right (Pressure sensitive) */
    protected final Trigger RT;

    /** D-Pad: Up */
    protected final Trigger DPU;

    /** D-Pad: Right */
    protected final Trigger DPR;

    /** D-Pad: Down */
    protected final Trigger DPD;

    /** D-Pad: Left */
    protected final Trigger DPL;

    /**
     * Constructs a DriverController on the specified HID port.
     *
     * @param port The USB port index on the Driver Station.
     */
    public DriverController(int port) {
        DriverStation.silenceJoystickConnectionWarning(true);
        controller = new CommandXboxController(port);
        A = controller.a();
        B = controller.b();
        X = controller.x();
        Y = controller.y();
        LB = controller.leftBumper();
        RB = controller.rightBumper();
        Back = controller.back();
        Start = controller.start();
        LS = controller.leftStick();
        RS = controller.rightStick();
        LT = controller.leftTrigger(0.25);
        RT = controller.rightTrigger(0.25);
        DPR = controller.povRight();
        DPD = controller.povDown();
        DPL = controller.povLeft();
        DPU = controller.povUp();
    }

    /** Debug modes for isolating subsystem testing during development. */
    public static enum DebugType {
        SHOOTER,
        SWERVE,
        INTAKE,
        INDEXER,
        CLIMBER;
    }

    /**
     * Binds specialized commands for debugging specific subsystems.
     *
     * @param subsystems The robot subsystems.
     * @param debugType The subsystem to debug.
     */
    public void bind(final Subsystems subsystems, DebugType debugType) {
        var swerve = subsystems.swerve;

        if (debugType == DebugType.SWERVE) {
            // Example: Bind X button to drive to a fixed field coordinate for swerve testing.
            this.X.whileTrue(Wayfinder.driveToTarget(swerve, new Pose2d(3, 1, new Rotation2d(0))));
        } else {
            System.out.println("DEBUG MODE: " + debugType + " (No specialized binds)");
        }
    }

    /**
     * The primary location for binding robot commands to driver controller inputs.
     *
     * <p>New programmers: Add your button bindings here using the {@code this.BUTTON.whileTrue()}
     * or {@code this.BUTTON.onTrue()} patterns.
     *
     * @param subsystems The robot subsystems available for command targeting.
     */
    public void bind(final Subsystems subsystems) {
        this.A.whileTrue(subsystems.turret.targetAngleCommand(Degrees.of(-180)));
        this.B.whileTrue(subsystems.turret.targetAngleCommand(Degrees.of(-90)));
        this.X.whileTrue(subsystems.turret.targetAngleCommand(Degrees.of(90)));
        this.Y.whileTrue(subsystems.turret.targetAngleCommand(Degrees.of(0)));
        // Example: this.A.whileTrue(new MyCommand(subsystems.mySubsystem));
        // Swerve driving is handled by the default command set in Robot.java,
        // so no explicit bind is needed here for basic teleop driving.
    }

    /**
     * Applies a deadband to a raw joystick value.
     *
     * @param supplier The raw value supplier.
     * @param deadband The deadband threshold.
     * @return A supplier that returns 0.0 if within the deadband, or a scaled value otherwise.
     */
    private DoubleSupplier deadbandSupplier(DoubleSupplier supplier, double deadband) {
        return () -> {
            double val = supplier.getAsDouble();
            if (Math.abs(val) > deadband) {
                if (val > 0.0) {
                    val = (val - deadband) / (1.0 - deadband);
                } else {
                    val = (val + deadband) / (1.0 - deadband);
                }
            } else {
                val = 0.0;
            }
            return val;
        };
    }

    /**
     * Returns the horizontal value of the right joystick.
     *
     * @return A supplier for Right X (-1.0 to 1.0).
     */
    public DoubleSupplier rightStickX() {
        return () -> -controller.getRightX();
    }

    /**
     * Returns the horizontal value of the right joystick with a deadband applied.
     *
     * @param deadband The deadband threshold.
     * @return A supplier for Right X.
     */
    public DoubleSupplier rightStickX(double deadband) {
        return deadbandSupplier(rightStickX(), deadband);
    }

    /**
     * Returns the vertical value of the right joystick.
     *
     * @return A supplier for Right Y (-1.0 to 1.0).
     */
    public DoubleSupplier rightStickY() {
        return controller::getRightY;
    }

    /**
     * Returns the vertical value of the right joystick with a deadband applied.
     *
     * @param deadband The deadband threshold.
     * @return A supplier for Right Y.
     */
    public DoubleSupplier rightStickY(double deadband) {
        return deadbandSupplier(rightStickY(), deadband);
    }

    /**
     * Returns the horizontal value of the left joystick.
     *
     * @return A supplier for Left X (-1.0 to 1.0).
     */
    public DoubleSupplier leftStickX() {
        return controller::getLeftX;
    }

    /**
     * Returns the horizontal value of the left joystick with a deadband applied.
     *
     * @param deadband The deadband threshold.
     * @return A supplier for Left X.
     */
    public DoubleSupplier leftStickX(double deadband) {
        return deadbandSupplier(leftStickX(), deadband);
    }

    /**
     * Returns the vertical value of the left joystick.
     *
     * @return A supplier for Left Y (-1.0 to 1.0).
     */
    public DoubleSupplier leftStickY() {
        return () -> -controller.getLeftY();
    }

    /**
     * Returns the vertical value of the left joystick with a deadband applied.
     *
     * @param deadband The deadband threshold.
     * @return A supplier for Left Y.
     */
    public DoubleSupplier leftStickY(double deadband) {
        return deadbandSupplier(leftStickY(), deadband);
    }

    /**
     * Returns the pressure value of the right trigger.
     *
     * @return A supplier for the axis value (0.0 to 1.0).
     */
    public DoubleSupplier rightTrigger() {
        return controller::getRightTriggerAxis;
    }

    /**
     * Returns the pressure value of the left trigger.
     *
     * @return A supplier for the axis value (0.0 to 1.0).
     */
    public DoubleSupplier leftTrigger() {
        return controller::getLeftTriggerAxis;
    }

    /**
     * Triggers a rumble effect on the controller.
     *
     * @param magnitude The intensity of the rumble (0.0 to 1.0).
     */
    public void rumble(double magnitude) {
        controller.getHID().setRumble(RumbleType.kBothRumble, magnitude);
    }
}
