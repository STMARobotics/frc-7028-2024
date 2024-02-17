package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController leftJockstick = new CommandXboxController(1);
  public MoveElevatorCommand(ElevatorSubsystem subsystem ) {
    elevatorSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // Will be to use controls later 
    elevatorSubsystem.moveToPosition(leftJockstick.getLeftY());
  }

}
