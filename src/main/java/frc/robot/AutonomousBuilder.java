package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoStartIntakeCommand;
import frc.robot.commands.EndShooterCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.ShootDonutCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousBuilder {
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ShooterSubsystem shooterSubsystem;

  public AutonomousBuilder(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem,
      ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  public Command endShooter() {
    return new EndShooterCommand(shooterSubsystem, indexerSubsystem);
  }

  public Command shootDonut() {
    return new ShootDonutCommand(shooterSubsystem, indexerSubsystem);
  }

  public Command deployIntake() {
    return new AutoStartIntakeCommand(intakeSubsystem, indexerSubsystem);
  }

  public Command moveElevator() {
    return new MoveElevatorCommand(elevatorSubsystem);
  }
}
