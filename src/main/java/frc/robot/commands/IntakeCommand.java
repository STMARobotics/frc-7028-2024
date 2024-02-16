package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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
    turretSubsystem.prepareToExchange();
  }
  
  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      intakeSubsystem.intake();
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
  }

}
