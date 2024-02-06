package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shooter motors
 */
public class RunShooterMotor extends Command {

  private static final double SHOOT_TIME = 0.25;
  private final ShooterSubsystem shooterSubsystem;
  private final Timer shootTimer = new Timer();
  protected double shooterRPS = 1;

  /**
   * Constructor
   * 
   * @param shooterRPS       velocity of shooter in rotations per second
   * @param shooterSubsystem shooter
   */
  public RunShooterMotor(double shooterRPS, ShooterSubsystem shooterSubsystem) {
    this.shooterRPS = shooterRPS;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.activeStop();
  }

  @Override
  public void execute() {
    shooterSubsystem.spinShooterWheel(shooterRPS);
    shootTimer.start();
  }

  @Override
  public boolean isFinished() {
    return shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.actuatorStop();
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}