package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpitCommand extends Command {

  private static final double SHOOT_TIME = 0.25;
  private final IntakeSubsystem intakeSubsystem;
  private final Timer shootTimer = new Timer();

  public SpitCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intakeSubsystem.Spit();
    shootTimer.start();
  }

  @Override
  public boolean isFinished() {
    return shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {

    shootTimer.stop();
  }
}