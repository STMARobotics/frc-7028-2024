package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to intake a note from the floor to the turret
 */
public class IntakeToTurretCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;

  public IntakeToTurretCommand(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, AmperSubsystem amperSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.amperSubsystem = amperSubsystem;

    addRequirements(intakeSubsystem, turretSubsystem, amperSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }
  
  @Override
  public void execute() {
    if (turretSubsystem.isInExchangePosition()) {
      intakeSubsystem.intake();
      amperSubsystem.intake();
      turretSubsystem.load();
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
    amperSubsystem.stop();
  }

}
