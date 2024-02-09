package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run shooter motors
 */
public class DeployIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final Timer deployTimer = new Timer();
  protected double shooterRPS = 1;

  public DeployIntakeCommand(double shooterRPS, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer) {
    this.intakeSubsystem = intakeSubsystem;
    indexerSubsystem = indexer;
  }

  @Override
  public void initialize() {
     deployTimer.start();
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();
  
    if (deployTimer.hasElapsed(1)) {
      intakeSubsystem.stop();
    } else {
      intakeSubsystem.deploy(Volts.of(3));
      intakeSubsystem.intakeRollers(Volts.of(3));
    }
  }

  @Override
  public boolean isFinished() {
    return !indexerSubsystem.shouldContinue();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    indexerSubsystem.stopIndexer();
    deployTimer.stop();
  }
}