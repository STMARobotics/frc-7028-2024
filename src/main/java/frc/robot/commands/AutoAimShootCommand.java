package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.SHOOT_TIME;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED;
import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimShootCommand extends Command {

  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final Pose2d poseSupplier;

  private final Timer shootTimer = new Timer();

  private Translation2d speakerTranslation;

  private final MutableMeasure<Angle> angleToRotate = MutableMeasure.zero(Rotations);

  public AutoAimShootCommand(ShooterSubsystem shooter,
  TurretSubsystem turretSubsystem, Pose2d poseSupplier) {
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.poseSupplier = poseSupplier;

    addRequirements(shooter, turretSubsystem);

  }

  @Override
  public void initialize() {
    shootTimer.reset();
    var alliance = DriverStation.getAlliance();
    speakerTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? SPEAKER_BLUE : SPEAKER_RED;
  }

  @Override
  public void execute() {
    var robotPose = poseSupplier;
    var robotTranslation = robotPose.getTranslation();

    // Angle to turn the robot. The shooter is on the back, so it's the angle to the speaker plus PI radians.
    var angleToSpeaker = speakerTranslation.minus(robotTranslation).getAngle().rotateBy(fromRadians(PI));
    angleToRotate.mut_replace(angleToSpeaker.minus(robotPose.getRotation()).getRotations(), Rotations);

    // Distance between the robot and the speaker
    var distanceToSpeaker = robotTranslation.getDistance(speakerTranslation);

    // Lookup shooter settings for this distance
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker);

    // Prepare shooter
    shooter.prepareToShoot(shootingSettings.getVelocity());

    // Prepare turret
    turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
    turretSubsystem.moveToYawPosition(angleToRotate);

    // When shooter is spun up and turret aimed, shoot and start timer
    if (shooter.isReadyToShoot() && turretSubsystem.isAtYawAndPitchTarget()) {
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
    shooter.stop();
    shootTimer.stop();
    turretSubsystem.stop();
  }

}
