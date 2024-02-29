package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Default command for the turret. This commands puts the turret in the exchange position and turns it off.
 */
public class DefaultTurretCommand extends Command {
  
  private final TurretSubsystem turretSubsystem;

  public DefaultTurretCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.stopRollers();
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      turretSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

}
