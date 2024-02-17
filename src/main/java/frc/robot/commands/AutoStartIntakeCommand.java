package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run shoot donut
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
    intakeSubsystem.runintakeRollers();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }
}