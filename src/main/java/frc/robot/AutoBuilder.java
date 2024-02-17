package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoBuilder {
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private ElevatorSubsystem elevatorSubsystem;


  public AutoBuilder(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
  }
  public Command deployIntake() {
    return new DeployIntakeCommand(intakeSubsystem, indexerSubsystem);
  }

  public Command moveElevator() {
    return new MoveElevatorCommand(elevatorSubsystem);
  }
} 
