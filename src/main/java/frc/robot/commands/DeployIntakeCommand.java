package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run shooter motors
 */
public class DeployIntakeCommand extends Command {

  private static final double DEPLOY_TIME = 0.25;
  private final IntakeSubsystem intakeSubsystem;
  private final Timer deployTimer = new Timer();
  protected double shooterRPS = 1;

  public DeployIntakeCommand(double shooterRPS, IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.deploy();
    deployTimer.start();
  }

  @Override
  public boolean isFinished() {
    return deployTimer.hasElapsed(DEPLOY_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    deployTimer.stop();
  }
}