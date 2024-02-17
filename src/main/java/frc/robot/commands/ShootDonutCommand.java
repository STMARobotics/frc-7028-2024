package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shoot donut
 */
public class ShootDonutCommand extends Command {

  private static final double SHOOT_TIME = 5;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final Timer shootTimer = new Timer();

  public ShootDonutCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
  }

  @Override
  public void execute() {
    indexerSubsystem.intake();
    shooterSubsystem.spinShooterWheel(40);
    if (shooterSubsystem.isShooterReady())
      indexerSubsystem.shoot();
      shootTimer.start();
  }

  @Override
  public boolean isFinished() {
    return (shootTimer.hasElapsed(SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stop();
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}