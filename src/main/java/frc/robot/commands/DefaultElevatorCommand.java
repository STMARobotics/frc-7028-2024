package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final BooleanSupplier safeToPark;

  public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier safeToPark) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.safeToPark = safeToPark;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    if (safeToPark.getAsBoolean()) {
      // Park the elevator if there's no interference (turret is clear)
      elevatorSubsystem.park(safeToPark);
    }
    if (elevatorSubsystem.isParked()) {
      // Turn the elevator off it it is parked
      elevatorSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

}
