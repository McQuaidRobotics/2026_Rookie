package igknighters.commands.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.Solver;
import igknighters.util.log.Log;
import java.util.function.Supplier;

public
class ShooterCommands { // tech for a real mech like for an og one but with yams the commands are in
    // class.
    public static Command setAngle(Shooter shooter, Angle angle) {
        return shooter.turret.run(() -> shooter.turret.targetAngle(angle));
    }

    public static Command faceForward(Shooter shooter) {
        return shooter.turret.run(
                () -> {
                    Pose2d robotPose = Robot.pose_pred.getPredictedPose();
                    Angle angle = robotPose.getRotation().getMeasure();
                    Angle targetAngle = angle.times(-1);
                    shooter.turret.targetAngle(targetAngle);
                });
    }

    public static Command homeHood(Shooter shooter) {
        // return Commands.run(() -> shooter.setHoodVoltage(-1)).until(()
        // ->shooter.isHoodSensorHit());
        return shooter.hood
                .run(() -> shooter.hood.setHoodVoltage(-3))
                .until(() -> shooter.isHoodSensorHit())
                .withTimeout(3.0)
                .withName("DRIVE DOWN HAS NOT HIT THE SENSOR YET HOME HOOD")
                .andThen(
                        Commands.either(
                                Commands.none(),
                                Commands.runOnce(
                                        () ->
                                                DriverStation.reportWarning(
                                                        "-------------------------------------THE"
                                                            + " HOOD WAS UNABLE TO TOUCH THE SENSOR"
                                                            + " PLEASE VERIFY THE SENSOR IS"
                                                            + " WORKING----------------------------------------------"
                                                            + " if this robot breaks cause u did"
                                                            + " not read this the programing team"
                                                            + " will murk you and we forfeit all"
                                                            + " responsibility dis ones on u bro or"
                                                            + " bra",
                                                        true)),
                                shooter::isHoodSensorHit))
                .andThen(
                        Commands.runOnce(
                                () -> {
                                    shooter.hood.setHoodVoltage(0);
                                    shooter.hood.zeroAt(Degrees.of(kHood.MIN_ANGLE_DEGREES));
                                }))
                .withName("HOOD IS DOWN ON SENSOR");
    }

    public static Command shoot(Shooter shooter, Indexer indexer) {
        return Commands.run(
                () -> {
                    Pose2d targetPose = ShootInformation.getInstance().getTargetPose().toPose2d();
                    ShooterState targetState =
                            Solver.solve(Robot.pose_pred.getPredictedPose(), targetPose);
                    ShooterState currentState = shooter.currentState(shooter);
                    shooter.targetState(targetState);
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/TARGET TURRET ANGLE:",
                            targetState.turretAngle.in(Degrees));
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/TARGET HOOD ANGLE:",
                            targetState.hoodAngle.in(Degrees));
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/TARGET RPM:",
                            targetState.flywheelVelocity.in(RPM));
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/CURRENT TURRET ANGLE:",
                            currentState.turretAngle.in(Degrees));
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/CURRENT HOOD ANGLE:",
                            currentState.hoodAngle.in(Degrees));
                    Log.log(
                            "ROBOT/SUBSYTEMS/SHOOTER/SHOOTER_COMMANDS/CURRENT RPM:",
                            currentState.flywheelVelocity.in(RPM));
                    if (Math.abs(
                                            targetState
                                                    .flywheelVelocity
                                                    .minus(currentState.flywheelVelocity)
                                                    .in(RPM))
                                    < 100
                            && Math.abs(
                                            targetState
                                                    .hoodAngle
                                                    .minus(currentState.hoodAngle)
                                                    .in(Degrees))
                                    < 5
                            && Math.abs(
                                            targetState
                                                    .turretAngle
                                                    .minus(currentState.turretAngle)
                                                    .in(Degrees))
                                    < 5) {
                        indexer.setStateNoCommand(RPM.of(3000), RPM.of(3000));
                    } else {
                        indexer.setStateNoCommand(RPM.of(0), RPM.of(0));
                    }
                },
                shooter.flyWheel,
                shooter.hood,
                shooter.turret,
                indexer.spindexer,
                indexer.exitRoller);
    }

    public static Command aim(Shooter shooter) {
        return Commands.run(
                () -> {
                    Pose2d targetPose = ShootInformation.getInstance().getTargetPose().toPose2d();
                    ShooterState targetState =
                            Solver.solve(Robot.pose_pred.getPredictedPose(), targetPose);
                    shooter.turret.targetAngle(targetState.turretAngle);
                },
                shooter.turret);
    }
}
