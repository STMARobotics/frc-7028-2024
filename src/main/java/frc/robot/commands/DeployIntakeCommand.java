package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run shooter motors
 */
public class DeployIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer) {
    this.intakeSubsystem = intakeSubsystem;
    indexerSubsystem = indexer;

    addRequirements(indexerSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();

    intakeSubsystem.deployIntake();
    intakeSubsystem.rollers();
  }

  @Override
  public boolean isFinished() {
    return indexerSubsystem.hasDonut();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.retractIntake();
    intakeSubsystem.stopRollers();
    indexerSubsystem.stopIndexer();
  }
}