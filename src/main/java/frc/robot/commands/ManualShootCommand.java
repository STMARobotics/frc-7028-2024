package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShootingConstants.SHOOT_TIME;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualShootCommand extends Command {
  
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final BooleanSupplier turretIsSafe;
  private final Measure<Velocity<Angle>> shooterVelocity;
  private final Measure<Angle> pitchAngle;
  private final Measure<Angle> yawAngle;

  private final Timer shootTimer = new Timer();

  public ManualShootCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
      BooleanSupplier turretIsSafe, Measure<Velocity<Angle>> shooterVelocity, Measure<Angle> pitchAngle,
      Measure<Angle> yawAngle) {
    
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretIsSafe = turretIsSafe;
    this.shooterVelocity = shooterVelocity;
    this.pitchAngle = pitchAngle;
    this.yawAngle = yawAngle;
  
    addRequirements(turretSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.prepareToShoot(shooterVelocity);
    turretSubsystem.moveToPitchPosition(pitchAngle);
    turretSubsystem.moveToShootingYawPosition(yawAngle, turretIsSafe);
    shootTimer.reset();
  }

  @Override
  public void execute() {
    if(shooterSubsystem.isReadyToShoot() && turretSubsystem.isAtYawAndPitchTarget()) {
      turretSubsystem.shoot();
      shootTimer.start();
    }
  }

  @Override
  public boolean isFinished() {
    return shootTimer.hasElapsed(SHOOT_TIME.in(Seconds));
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.prepareToExchange();
    shooterSubsystem.stop();
    shootTimer.stop();
  }

}
