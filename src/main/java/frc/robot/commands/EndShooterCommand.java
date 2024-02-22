package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EndShooterCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public EndShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooterSubsystem.stop();
    indexerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }
}