package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.ROBOT_ROTATION_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.ROBOT_SPEED_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_RATE_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_SHOOT_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_SHOOT_LIMIT_REVERSE;
import static java.lang.Math.PI;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This command automatically scores in the speaker.
 */
public class ScoreSpeakerCommand extends Command {

  // Forward and reverse targets for the drivetrain when the turret is out of range
  // They're a few degrees (DRIVETRAIN_MARGIN) inside the turret shooting limits to avoid getting stuck on the edge
  private static final Measure<Angle> DRIVETRAIN_MARGIN = Degrees.of(5);
  private static final Rotation2d YAW_LIMIT_FORWARD = new Rotation2d(YAW_SHOOT_LIMIT_FORWARD.minus(DRIVETRAIN_MARGIN));
  private static final Rotation2d YAW_LIMIT_REVERSE = new Rotation2d(YAW_SHOOT_LIMIT_REVERSE.plus(DRIVETRAIN_MARGIN));

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier turretIsSafe;

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

  public ScoreSpeakerCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem, LEDSubsystem ledSubsystem, BooleanSupplier turretIsSafe) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.turretIsSafe = turretIsSafe;

    addRequirements(drivetrain, shooter, turretSubsystem);

    swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
    swerveRequestFacing.HeadingController = new PhoenixPIDController(THETA_kP, THETA_kI, THETA_kD);
    swerveRequestFacing.HeadingController.enableContinuousInput(-PI, PI);
    swerveRequestFacing.HeadingController.setTolerance(AIM_TOLERANCE.in(Radians));

    swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
  }

  @Override
  public void initialize() {
    swerveRequestFacing.HeadingController.reset();
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

    // Calculate the angle to the speaker
    var angleToSpeaker = speakerTranslation.minus(robotTranslation).getAngle();
    
    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(angleToSpeaker.minus(robotPose.getRotation()).getRotations(), Rotations);
    
    // Calculate ready state
    var isShooterReady = shooter.isReadyToShoot();
    var isInTurretRange = TurretSubsystem.isYawInShootingRange(turretYawTarget);


    // Aim drivetrain
    // NOTE: Slew rate limit needs to be applied so the robot slows properly (see 2022 robot doing "stoppies")
    if (isInTurretRange) {
        // Prepare shooter
      shooter.prepareToShoot(shootingSettings.getVelocity());
    
      // Set the turret position
      turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
      turretSubsystem.moveToYawPosition(turretYawTarget, turretIsSafe);

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

      // Decide the direction to turn, then set the robot rotation target so the turret's shooting yaw limit on that
      // side is pointed at the speaker
      Rotation2d robotTargetDirection = angleToSpeaker.minus(fromRadians(PI)); // Turret is on the back of the robot
      if (robotTargetDirection.minus(robotPose.getRotation()).getRadians() > 0) {
        robotTargetDirection = robotTargetDirection.minus(YAW_LIMIT_FORWARD);
      } else {
        robotTargetDirection = robotTargetDirection.minus(YAW_LIMIT_REVERSE);
      }

      // If the drivetrain is getting close, start getting ready to shoot
      if (Math.abs(robotPose.getRotation().minus(robotTargetDirection).getDegrees()) < 25) {
        // Prepare the shooter
        shooter.prepareToShoot(shootingSettings.getVelocity());
        
        // Set the turret position
        turretSubsystem.moveToYawPosition(turretYawTarget, turretIsSafe);
        turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
      }

      drivetrain.setControl(swerveRequestFacing
          .withVelocityX(limitedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(limitedChassisSpeeds.vyMetersPerSecond)
          .withTargetDirection(robotTargetDirection));
    }

    var isPitchReady = turretSubsystem.isAtPitchTarget();
    var isYawReady = turretSubsystem.isAtYawTarget();
    if (isShooterReady && isRobotStopped() && isPitchReady && isYawReady) {
      // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot and start timer
      turretSubsystem.shoot();
      isShooting = true;
    }

    // Update LEDs with ready state
    if (isShooting) {
      ledSubsystem.setUpdater(l -> l.setAll(kGreen));
    } else {
      ledSubsystem.setUpdater(l -> 
          l.setLEDSegments(NOTE_COLOR, isShooterReady, isInTurretRange, isPitchReady, isYawReady));
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
    return !turretSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    turretSubsystem.prepareToExchange();
    drivetrain.setControl(new SwerveRequest.Idle());
    ledSubsystem.setUpdater(null);
  }

}
