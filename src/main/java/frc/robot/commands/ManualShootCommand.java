package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShootingConstants.SHOOT_TIME;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/** Command to manually shoot with predefined settings */
public class ManualShootCommand extends Command {

  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final AngularVelocity shooterVelocity;
  private final Angle pitchAngle;
  private final Angle yawAngle;

  private final Timer shootTimer = new Timer();

  public ManualShootCommand(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      AngularVelocity shooterVelocity,
      Angle pitchAngle,
      Angle yawAngle) {

    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterVelocity = shooterVelocity;
    this.pitchAngle = pitchAngle;
    this.yawAngle = yawAngle;

    addRequirements(turretSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.prepareToShoot(shooterVelocity);
    turretSubsystem.moveToPitchPosition(pitchAngle);
    turretSubsystem.moveToShootingYawPosition(yawAngle);
    shootTimer.reset();
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isReadyToShoot() && turretSubsystem.isAtYawAndPitchTarget()) {
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
    turretSubsystem.prepareToIntake();
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}
