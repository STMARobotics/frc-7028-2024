package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Testing command for tuning speaker shots. This is not intended to be used in-game. This command reads from the
 * NetworkTables to get shooter pitch, velocity, and yaw.
 */
public class TuneSpeakerCommand extends Command {
  
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final BooleanSupplier turretIsSafe;

  private final DoubleEntry pitchSubscriber;
  private final DoubleEntry velocitySubscriber;
  private final DoubleEntry yawSubscriber;

  private boolean shooting = false;

  public TuneSpeakerCommand(
      TurretSubsystem turretSubsystem,
      AmperSubsystem amperSubsystem,
      ShooterSubsystem shooterSubsystem, 
      BooleanSupplier turretIsSafe) {
    
    this.turretSubsystem = turretSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretIsSafe = turretIsSafe;

    var nt = NetworkTableInstance.getDefault();
    var table = nt.getTable("Tune Shoot");
    pitchSubscriber = table.getDoubleTopic("Pitch (degrees)").getEntry(0.0);
    pitchSubscriber.set(0.0);
    velocitySubscriber = table.getDoubleTopic("Velocity (RPS)").getEntry(0.0);
    velocitySubscriber.set(0.0);
    yawSubscriber = table.getDoubleTopic("Yaw (Degrees)").getEntry(180.0);
    yawSubscriber.set(180.0);

    addRequirements(turretSubsystem, amperSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooting = false;
  }
  
  @Override
  public void execute() {
    turretSubsystem.moveToPitchPosition(Degrees.of(pitchSubscriber.get(0.0)));
    turretSubsystem.moveToYawShootingPosition(Degrees.of(yawSubscriber.get(180.0)), turretIsSafe);
    shooterSubsystem.prepareToShoot(RotationsPerSecond.of(velocitySubscriber.get(10)));
    if (shooting || (turretSubsystem.isAtYawAndPitchTarget() && shooterSubsystem.isReadyToShoot())) {
      turretSubsystem.shoot();
      shooting = true;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
    amperSubsystem.stop();
    shooterSubsystem.stop();
  }
}
