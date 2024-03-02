package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Runs everything in reverse to attempt to eject a jammed note
 */
public class EjectIntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private AmperSubsystem amperSubsystem;
  private TurretSubsystem turretSubsystem;

  public EjectIntakeCommand(IntakeSubsystem intakeSubsystem, AmperSubsystem amperSubsystem,
      TurretSubsystem turretSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(intakeSubsystem, amperSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      amperSubsystem.load();
      intakeSubsystem.reverse();
      turretSubsystem.eject();
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    intakeSubsystem.stop();
    turretSubsystem.stop();
  }
  
}
