package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to score in the amp, when there is already a note in the amper.
 */
public class ScoreAmpCommand extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final AmperSubsystem amperSubsystem;

  public ScoreAmpCommand(ElevatorSubsystem elevatorSubsystem, AmperSubsystem amperSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.amperSubsystem = amperSubsystem;

    addRequirements(elevatorSubsystem, amperSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.prepareToAmp();
  }

  @Override
  public void execute() {
    if (elevatorSubsystem.isAtTarget()) {
      amperSubsystem.score();
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    elevatorSubsystem.stop();
  }
}
