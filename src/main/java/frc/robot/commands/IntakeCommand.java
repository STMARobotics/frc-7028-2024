package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem; 
  private IndexerSubsystem indexerSubsystem;

  public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
    intakeSubsystem = intake;
    indexerSubsystem = indexer;
  }

  @Override
  public void execute() {
    intakeSubsystem.Intake();
    indexerSubsystem.intake();
  }

  @Override
  public boolean isFinished() {
    return !indexerSubsystem.shouldContinueRunning();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.Stop();
  }
}
