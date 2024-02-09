package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private final CommandJoystick leftJoystick = new CommandJoystick(1);

  public MoveElevatorCommand(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(leftJoystick.getY());
  }

}
