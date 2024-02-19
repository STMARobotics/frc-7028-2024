package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToAmperCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final AmperSubsystem amperSubsystem;

  public IntakeToAmperCommand(IntakeSubsystem intakeSubsystem, AmperSubsystem amperSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;

    addRequirements(intakeSubsystem, amperSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.intake();
    amperSubsystem.load();
  }

  @Override
  public boolean isFinished() {
    // TODO stop when we have a sensor to see detect gamepiece
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    amperSubsystem.stop();
  }

}
