package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to intake a note from the floor to the turret
 */
public class IntakeCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TurretSubsystem turretSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(intakeSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToIntake();
  }
  
  @Override
  public void execute() {
    if (turretSubsystem.isInIntakePosition()) {
      intakeSubsystem.intake();
      turretSubsystem.intake();
    }
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    turretSubsystem.stopPitch();
    turretSubsystem.stopYaw();
    turretSubsystem.runRollers(RotationsPerSecond.zero());
  }

}
