package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to score in the amp, when there is already a note in the amper.
 */
public class ScoreAmpCommand extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final TurretSubsystem turretSubsystem;

  public ScoreAmpCommand(ElevatorSubsystem elevatorSubsystem, AmperSubsystem amperSubsystem, TurretSubsystem turretSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(elevatorSubsystem, amperSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();

  }

  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      elevatorSubsystem.prepareToAmp();
      if (elevatorSubsystem.isAtTarget()) {
        amperSubsystem.score();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    elevatorSubsystem.park();
    turretSubsystem.stop();
  }
}
