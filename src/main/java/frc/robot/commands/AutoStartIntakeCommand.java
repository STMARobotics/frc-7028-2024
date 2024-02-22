package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to start the intake during auto, 
 * end intake is not needed because the command is cancelled 
 * once the path its scheduled on finishes.
 */

public class AutoStartIntakeCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public AutoStartIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }

}
