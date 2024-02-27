package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to score in the trap, when there is already a note in the amper, and the robot is hanging on the chain.
 */
public class ScoreTrapCommand extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final TurretSubsystem turretSubsystem;

  public ScoreTrapCommand(
        ElevatorSubsystem elevatorSubsystem, AmperSubsystem amperSubsystem, TurretSubsystem turretSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(elevatorSubsystem, amperSubsystem, turretSubsystem);
  }

  @Override
  public void execute() {
    // Move the turret into position
    turretSubsystem.prepareToTrap();

    if (turretSubsystem.isInTrapPosition()) {
      // Move the elevator only after the turret is in position, otherwise they collide
      elevatorSubsystem.prepareToTrap();
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    elevatorSubsystem.stop();
    turretSubsystem.stop();
  }
}
