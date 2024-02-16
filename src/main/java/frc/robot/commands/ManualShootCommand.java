package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shoot donut
 */
public class ManualShootCommand extends Command {

  private static final double SHOOT_TIME = 5;
  private final ShooterSubsystem shooterSubsystem;

  private final Timer shootTimer = new Timer();

  public ManualShootCommand(ShooterSubsystem shooterSubsystem) {

    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
  }

  @Override
  public void execute() {
    shooterSubsystem.spinShooterWheel();
    if (shooterSubsystem.isShooterReady())
      shooterSubsystem.runIntake();
      shootTimer.start();
  }

  @Override
  public boolean isFinished() {
    return (shootTimer.hasElapsed(SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}