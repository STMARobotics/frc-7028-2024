package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeRing extends Command {

  public IntakeSubsystem intakeSubsystem; 
  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  @Override
  public void execute() {
    intakeSubsystem.Intake();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }
}
