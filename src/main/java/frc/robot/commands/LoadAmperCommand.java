package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LoadAmperCommand extends Command {
  private final AmperSubsystem amperSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  public LoadAmperCommand(AmperSubsystem amperSubsystem, TurretSubsystem turretSubsystem,
    IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(amperSubsystem, turretSubsystem, intakeSubsystem, elevatorSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    elevatorSubsystem.prepareToExchangeToAmper(turretSubsystem::isClearOfElevator);
    if (turretSubsystem.isInExchangePosition() && elevatorSubsystem.isAtTarget()) {
      amperSubsystem.load();
      turretSubsystem.eject();
      intakeSubsystem.intake();
    }
  }

  @Override
  public boolean isFinished() {
    return amperSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.runRollers(RotationsPerSecond.zero());
    turretSubsystem.stopYaw();
    turretSubsystem.stopPitch();
    turretSubsystem.runRollers(RotationsPerSecond.zero());
    intakeSubsystem.stop();
    elevatorSubsystem.stop();
  }
  
}
