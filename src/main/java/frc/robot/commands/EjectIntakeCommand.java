package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Runs everything in reverse to attempt to eject a jammed note
 */
public class EjectIntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private AmperSubsystem amperSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;

  public EjectIntakeCommand(IntakeSubsystem intakeSubsystem, AmperSubsystem amperSubsystem,
      TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, amperSubsystem, turretSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    if (turretSubsystem.isInExchangePosition()) {
      amperSubsystem.load();
      intakeSubsystem.reverse();
      turretSubsystem.eject();
      shooterSubsystem.reverse();
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    intakeSubsystem.stop();
    turretSubsystem.stop();
    shooterSubsystem.stop();
  }
  
}
