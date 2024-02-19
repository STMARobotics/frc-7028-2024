package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class IntakeCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, AmperSubsystem amperSubsystem) {
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
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      intakeSubsystem.intake();
      amperSubsystem.intake();
      turretSubsystem.load();
    }
  }

  @Override
  public boolean isFinished() {
    // TODO stop when we have a sensor to see detect gamepiece
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    turretSubsystem.stop();
    amperSubsystem.stop();
  }

}
