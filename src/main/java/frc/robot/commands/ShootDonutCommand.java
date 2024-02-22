package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shoot donut
 */
public class ShootDonutCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public ShootDonutCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooterSubsystem.spinShooterWheel(45);
    if (shooterSubsystem.isShooterReady())
      indexerSubsystem.shoot();
  }

  @Override
  public boolean isFinished() {
    return !indexerSubsystem.hasDonut();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    shooterSubsystem.stop();

  }
}