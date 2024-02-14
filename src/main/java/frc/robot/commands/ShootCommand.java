package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;
import static frc.robot.Constants.ShootingConstants.SHOOT_TIME;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED;
import static java.lang.Math.PI;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final IndexerSubsystem indexer;
  private final ShooterSubsystem shooter;

  private final Timer shootTimer = new Timer();

  private Translation2d speakerTranslation;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  public ShootCommand(CommandSwerveDrivetrain drivetrain, IndexerSubsystem indexer, ShooterSubsystem shooter) {
    this.drivetrain = drivetrain;
    this.indexer = indexer;
    this.shooter = shooter;

    addRequirements(drivetrain, indexer, shooter);

    swerveRequest.ForwardReference = ForwardReference.RedAlliance;
    swerveRequest.HeadingController = new PhoenixPIDController(THETA_kP, THETA_kI, THETA_kD);
    swerveRequest.HeadingController.enableContinuousInput(-PI, PI);
  }

  @Override
  public void initialize() {
    swerveRequest.HeadingController.reset();
    shootTimer.reset();
    var alliance = DriverStation.getAlliance();
    speakerTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? SPEAKER_BLUE : SPEAKER_RED;
  }

  @Override
  public void execute() {
    var robotPose = drivetrain.getState().Pose;
    var robotTranslation = robotPose.getTranslation();

    // Distance between the robot and the speaker
    var distanceToSpeaker = robotTranslation.getDistance(speakerTranslation);

    // Angle to turn the robot. The shooter is on the back, so it's the angle to the speaker plus PI radians.
    var angleToSpeaker = speakerTranslation.minus(robotTranslation).getAngle().rotateBy(fromRadians(PI));

    // Prepare shooter
    shooter.prepareToShoot(RotationsPerSecond.of(40), Rotations.of(0.35));
    // Aim drivetrain
    drivetrain.setControl(swerveRequest.withTargetDirection(angleToSpeaker));

    // When shooter is spun up, and drivetrain aimed, shoot and run timer
    if (shooter.isReadyToShoot() && Math.abs(robotPose.getRotation().minus(angleToSpeaker).getDegrees()) < 3) {
      indexer.shoot();
      shootTimer.start();
    }
  }

  @Override
  public boolean isFinished() {
    return shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
    shootTimer.stop();
  }

}
