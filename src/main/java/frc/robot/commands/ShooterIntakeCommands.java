package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IndexPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * Static factory class for commands involving shooter and intake subsystems
 */
public class ShooterIntakeCommands {

  //Internal method to generate the command sequence for feeding the shooter balls based on the current ball index
  private static Command feedShooterCommand(Intake intake) {
    return new ConditionalCommand(
      intake.upperFeedCommand()
        .andThen(intake.indexBallsCommand())
        .andThen(intake.upperFeedCommand()),
      intake.upperFeedCommand(),
      () -> intake.getBallIndex(IndexPosition.LOWER));
  }

  //Shooter sequence command
  public static Command shootCommand(boolean high, Shooter shooter, Intake intake){
    return shooter.shooterSpinUpCommand(high ? Constants.Shooter.Velocities.highDefault:Constants.Shooter.Velocities.lowDefault)
      .andThen(feedShooterCommand(intake))
      .andThen(shooter.shooterIdleCommand());
  }

  public static Command defaultedShootCommand(Shooter shooter, Intake intake){
    return shooter.shooterSpinUpCommand(Constants.Shooter.Velocities.lowDefault)
      .andThen(new StartEndCommand(
        () -> {
          intake.setMotor(IndexPosition.UPPER, Constants.Intake.OutputLevel.shooterFeedVoltage);
          intake.setMotor(IndexPosition.LOWER, Constants.Intake.OutputLevel.shooterFeedVoltage);
        },
        () -> {
          intake.setMotor(IndexPosition.UPPER, 0);
          intake.setMotor(IndexPosition.LOWER, 0);
          shooter.setVelocity(Constants.Shooter.Velocities.idleVelocity);
        },
        shooter, intake)
      );
  }

  //Eject ball from shooter sequence
  public static Command ejectUpperCommand(Shooter shooter, Intake intake){
    return shooter.shooterSpinUpCommand(Constants.Shooter.Velocities.ejectVelocity)
      .andThen(new ConditionalCommand(
        intake.upperFeedCommand().andThen(intake.indexBallsCommand()),
        intake.upperFeedCommand(),
        () -> intake.getBallIndex(IndexPosition.LOWER)
      ))
      .andThen(shooter.shooterIdleCommand());
  }
}
