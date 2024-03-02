package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class BabyBirdCommand extends Command {
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final BooleanSupplier turretIsSafe;
  private boolean sensorTripped = false;
  private boolean sensorCleared = false;

  public BabyBirdCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem, BooleanSupplier turretIsSafe) {
    this.turretSubsystem = turretSubsystem;
    this.turretIsSafe = turretIsSafe;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(turretSubsystem, shooterSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.moveToYawPosition(Degrees.of(180), turretIsSafe);
    turretSubsystem.moveToPitchPosition(Rotations.of(0.081));
    turretSubsystem.runRollers(RotationsPerSecond.of(-25));
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
      turretSubsystem.load();
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