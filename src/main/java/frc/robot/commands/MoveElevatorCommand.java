
package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private final CommandJoystick leftJoystick = new CommandJoystick(1);
  private final BooleanSupplier turretIsClear;


  public MoveElevatorCommand(
    ElevatorSubsystem elevatorSubsystem,
    BooleanSupplier turretIsClear){
      this.elevatorSubsystem = elevatorSubsystem;
      this.turretIsClear = turretIsClear;

      addRequirements(elevatorSubsystem);
    }

  @Override
  public void execute() {
    if (turretIsClear.getAsBoolean()){
          elevatorSubsystem.moveToPosition(leftJoystick.getY());
    }
    else{
          elevatorSubsystem.stop();
    }
  }
}
