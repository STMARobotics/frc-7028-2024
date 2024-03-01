package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.ROBOT_ROTATION_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.ROBOT_SPEED_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.SHOOTER_COEFFICIENT;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.SHOOT_TIME;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_RATE_LIMIT;
import static java.lang.Math.PI;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This command automatically scores in the speaker.
 */
public class ScoreSpeakerMovingCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier turretIsSafe;

  private final Timer shootTimer = new Timer();

  private final ChassisSpeedsRateLimiter rateLimiter = new ChassisSpeedsRateLimiter(
      TRANSLATION_RATE_LIMIT.in(MetersPerSecondPerSecond), ROTATION_RATE_LIMIT.in(RadiansPerSecond.per(Second)));

  // Reusable objects to prevent reallocation (to reduce memory pressure)
  private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private final MutableMeasure<Angle> turretYawTarget = MutableMeasure.zero(Rotations);

  private Translation2d speakerTranslation;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);
  
  private final SwerveRequest.FieldCentric swerveRequestRotation = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);

  private boolean isShooting = false;

  public ScoreSpeakerMovingCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem, LEDSubsystem ledSubsystem, BooleanSupplier turretIsSafe) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.turretIsSafe = turretIsSafe;

    addRequirements(shooter, turretSubsystem);

    swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
    swerveRequestFacing.HeadingController = new PhoenixPIDController(THETA_kP, THETA_kI, THETA_kD);
    swerveRequestFacing.HeadingController.enableContinuousInput(-PI, PI);
    swerveRequestFacing.HeadingController.setTolerance(AIM_TOLERANCE.in(Radians));

    swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
  }

  @Override
  public void initialize() {
    swerveRequestFacing.HeadingController.reset();
    shootTimer.reset();
    rateLimiter.reset(drivetrain.getCurrentFieldChassisSpeeds());
    var alliance = DriverStation.getAlliance();
    speakerTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? SPEAKER_BLUE : SPEAKER_RED;
    isShooting = false;
  }

  @Override
  public void execute() {
    var robotPose = drivetrain.getState().Pose;
    var robotTranslation = robotPose.getTranslation();

    // Distance between the robot and the speaker
    var distanceToSpeaker = robotTranslation.getDistance(speakerTranslation);

    // Lookup shooter settings for this distance
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker);

    // Calculate time to hit speaker
    var timeUntilScored = SHOOTER_COEFFICIENT * distanceToSpeaker * SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker).getVelocity().in(RotationsPerSecond);

    // Calculate the predicted offset of the speaker compared to current pose (in meters)
    var speakerPredictedOffset = new Translation2d((drivetrain.getState().speeds.vxMetersPerSecond * timeUntilScored), 
    (drivetrain.getState().speeds.vyMetersPerSecond * timeUntilScored));

    // Calculate the angle to the speaker
    var angleToSpeaker = speakerTranslation.minus(robotTranslation).minus(speakerPredictedOffset).getAngle();
    
    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(angleToSpeaker.minus(robotPose.getRotation()).getRotations(), Rotations);
    
    // Prepare shooter
    shooter.prepareToShoot(shootingSettings.getVelocity());
    
    // Move the turret
    turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
    turretSubsystem.moveToYawShootingPosition(turretYawTarget, turretIsSafe);

    // Calculate ready state and send to telemetry
    var isShooterReady = shooter.isReadyToShoot();
    var isInTurretRange = TurretSubsystem.isYawInShootingRange(turretYawTarget);
    var isPitchReady = turretSubsystem.isAtPitchTarget();
    var isYawReady = turretSubsystem.isAtYawTarget();
    
    // Update LEDs with ready state
    if (isShooting) {
      ledSubsystem.setUpdater(l -> l.setAll(kGreen));
    } else {
      ledSubsystem.setUpdater(l -> 
          l.setLEDSegments(NOTE_COLOR, isShooterReady, isInTurretRange, isPitchReady, isYawReady));
    }

    // Aim drivetrain
    // NOTE: Slew rate limit needs to be applied so the robot slows properly (see 2022 robot doing "stoppies")
    if (isInTurretRange) {
      // Turret can reach, stop robot
      chassisSpeeds.vxMetersPerSecond = 0.0;
      chassisSpeeds.vyMetersPerSecond = 0.0;
      var limitedChassisSpeeds = rateLimiter.calculate(chassisSpeeds);
      drivetrain.setControl(swerveRequestRotation
          .withVelocityX(limitedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(limitedChassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(0.0));
    } else {
      // Turret cannot reach, turn robot
      chassisSpeeds.vxMetersPerSecond = 0.0;
      chassisSpeeds.vyMetersPerSecond = 0.0;
      var limitedChassisSpeeds = rateLimiter.calculate(chassisSpeeds);

      // Turn toward the *back of the robot* toward the target because that's where the turret is
      drivetrain.setControl(swerveRequestFacing
          .withVelocityX(limitedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(limitedChassisSpeeds.vyMetersPerSecond)
          .withTargetDirection(angleToSpeaker.rotateBy(fromRadians(PI))));
    }

    if (isShooterReady && isRobotStopped() && isPitchReady && isYawReady) {
      // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot and start timer
      turretSubsystem.shoot();
      shootTimer.start();
      ledSubsystem.setUpdater(l -> l.setAll(Color.kGreen));
      isShooting = true;
    }
  }

  /**
   * Checks if the robot is moving slow enough to allow shooting.
   * @return true if the robot is moving slow enough to shoot, otherwise false
   */
  private boolean isRobotStopped() {
    var currentSpeeds = drivetrain.getCurrentFieldChassisSpeeds();
    return new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond).getNorm()
        < ROBOT_SPEED_TOLERANCE.in(MetersPerSecond)
        && Math.abs(currentSpeeds.omegaRadiansPerSecond) < ROBOT_ROTATION_TOLERANCE.in(RadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    return shootTimer.hasElapsed(SHOOT_TIME.in(Seconds));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    shootTimer.stop();
    turretSubsystem.prepareToExchange();
    drivetrain.setControl(new SwerveRequest.Idle());
    ledSubsystem.setUpdater(null);
  }

}
