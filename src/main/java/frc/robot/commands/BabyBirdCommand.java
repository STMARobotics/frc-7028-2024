package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to "feed the robot shooter" from the human player station. Tilts the shooter up so the
 * human player can drop a note in.
 */
public class BabyBirdCommand extends Command {

  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  private boolean sensorTripped = false;
  private boolean sensorCleared = false;

  public BabyBirdCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(turretSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToBabyBird();
    shooterSubsystem.reverse();
    sensorTripped = false;
    sensorCleared = false;
  }

  @Override
  public void execute() {
    if (turretSubsystem.hasNote()) {
      sensorTripped = true;
    }
    if (sensorTripped && !turretSubsystem.hasNote()) {
      turretSubsystem.intake();
      shooterSubsystem.stop();
      sensorCleared = true;
    }
  }

  @Override
  public boolean isFinished() {
    return sensorTripped && sensorCleared && turretSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopPitch();
    turretSubsystem.stopYaw();
    turretSubsystem.runRollers(RotationsPerSecond.zero());
    shooterSubsystem.stop();
  }
}
