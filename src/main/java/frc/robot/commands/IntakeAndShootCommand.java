package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class IntakeAndShootCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public IntakeAndShootCommand(IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, AmperSubsystem amperSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, turretSubsystem, amperSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
    turretSubsystem.moveToPitchPosition(Rotations.of(.01));
  }
  
  @Override
  public void execute() {
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      intakeSubsystem.intake();
      amperSubsystem.intake();
      turretSubsystem.load();
      shooterSubsystem.spinShooterWheels(RotationsPerSecond.of(70));
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
    shooterSubsystem.stop();
  }

}
