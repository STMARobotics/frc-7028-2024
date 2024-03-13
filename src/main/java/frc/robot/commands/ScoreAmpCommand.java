package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to score in the amp, when there is already a note in the turret.
 */
public class ScoreAmpCommand extends Command {
  
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ScoreAmpCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(turretSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToAmp();
    shooterSubsystem.prepareToAmp();
  }

  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget() && shooterSubsystem.isReadyToShoot()) {
      turretSubsystem.shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    shooterSubsystem.stop();
  }
}
