package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
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
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;

  private final Timer shootTimer = new Timer();

  private Translation2d speakerTranslation;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  public ShootCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;

    addRequirements(drivetrain, shooter);

    swerveRequest.ForwardReference = ForwardReference.RedAlliance;
    swerveRequest.HeadingController = new PhoenixPIDController(THETA_kP, THETA_kI, THETA_kD);
    swerveRequest.HeadingController.enableContinuousInput(-PI, PI);
    swerveRequest.HeadingController.setTolerance(AIM_TOLERANCE.in(Radians));
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

    // Lookup shooter settings for this distance
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker);

    // Angle to turn the robot. The shooter is on the back, so it's the angle to the speaker plus PI radians.
    var angleToSpeaker = speakerTranslation.minus(robotTranslation).getAngle().rotateBy(fromRadians(PI));

    // Prepare shooter
    shooter.prepareToShoot(shootingSettings.getVelocity());
    // TODO prepare turret
    
    // Aim drivetrain
    drivetrain.setControl(swerveRequest.withTargetDirection(angleToSpeaker));

    // When shooter is spun up and drivetrain aimed, shoot and run timer
    if (shooter.isReadyToShoot() && swerveRequest.HeadingController.atSetpoint()) {
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
  }

}
