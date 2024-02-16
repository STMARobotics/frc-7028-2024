package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LoadAmperCommand extends Command {
  private final AmperSubsystem amperSubsystem;
  private final TurretSubsystem turretSubsystem;

  public LoadAmperCommand(AmperSubsystem amperSubsystem, TurretSubsystem turretSubsystem) {
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(amperSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      amperSubsystem.load();
      turretSubsystem.eject();
    }
  }

  @Override
  public boolean isFinished() {
    // TODO stop when we have a sensor to know a note is in the amper
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    turretSubsystem.stop();
  }
  
}
