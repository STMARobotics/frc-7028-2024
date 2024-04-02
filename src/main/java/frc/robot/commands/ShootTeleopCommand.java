package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;
import static frc.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.DRIVETRAIN_YAW_LIMIT_FORWARD;
import static frc.robot.Constants.ShootingConstants.DRIVETRAIN_YAW_LIMIT_REVERSE;
import static frc.robot.Constants.ShootingConstants.SHOOT_WHILE_MOVING_COEFFICIENT;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_RATE_LIMIT;
import static java.lang.Math.PI;

import java.util.function.Supplier;

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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.VelocityPitchInterpolator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This command automatically shoots at a target while a supplier (the driver) is translating the robot. This command
 * will slow translation, and take of rotation to make sure the turret can reach the target.
 */
public class ShootTeleopCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final LEDSubsystem ledSubsystem;

  private final Supplier<Measure<Velocity<Distance>>> xSupplier;
  private final Supplier<Measure<Velocity<Distance>>> ySupplier;

  private final ChassisSpeedsRateLimiter rateLimiter = new ChassisSpeedsRateLimiter(
      TRANSLATION_RATE_LIMIT.in(MetersPerSecondPerSecond), ROTATION_RATE_LIMIT.in(RadiansPerSecond.per(Second)));
    
  // Reusable objects to prevent reallocation (to reduce memory pressure)
  private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private final MutableMeasure<Angle> turretYawTarget = MutableMeasure.zero(Rotations);
  private final Translation2d targetRed;
  private final Translation2d targetBlue;
  private final VelocityPitchInterpolator lookupTable;

  private Translation2d targetTranslation;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);
  
  private final SwerveRequest.FieldCentric swerveRequestRotation = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);

  private boolean isShooting = false;

  public ShootTeleopCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem, LEDSubsystem ledSubsystem,
      Supplier<Measure<Velocity<Distance>>> xSupplier, Supplier<Measure<Velocity<Distance>>> ySupplier,
      Translation2d targetRed, Translation2d targetBlue, VelocityPitchInterpolator lookupTable) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetRed = targetRed;
    this.targetBlue = targetBlue;
    this.lookupTable = lookupTable;

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
    targetTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? targetBlue : targetRed;
    isShooting = false;
  }

  @Override
  public void execute() {
    var robotPose = drivetrain.getState().Pose;
    
    // Translation to the center of the turret
    var turretTranslation = TurretSubsystem.getTurretTranslation(robotPose);

    // Distance between the robot and the target
    var distanceToTarget = turretTranslation.getDistance(targetTranslation);

    // Lookup shooter settings for this distance
    var shootingSettings = lookupTable.calculate(distanceToTarget);

    // Calculate time to hit target
    var timeUntilScored = 
        SHOOT_WHILE_MOVING_COEFFICIENT * (distanceToTarget / shootingSettings.getVelocity().in(RotationsPerSecond));

    // Calculate the predicted offset of the target compared to current pose (in meters)
    var targetPredictedOffset =
        new Translation2d((drivetrain.getCurrentFieldChassisSpeeds().vxMetersPerSecond * timeUntilScored), 
            (drivetrain.getCurrentFieldChassisSpeeds().vyMetersPerSecond * timeUntilScored));

    var predictedTargetTranslation = targetTranslation.minus(targetPredictedOffset);

    var predictedDist = predictedTargetTranslation.getDistance(turretTranslation);

    // Calculate the angle to the target
    var angleToTarget = predictedTargetTranslation.minus(turretTranslation).getAngle();
    
    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(angleToTarget.minus(robotPose.getRotation()).getRotations(), Rotations);

    shootingSettings = lookupTable.calculate(predictedDist);
    
    // Calculate ready state
    var isShooterReady = shooter.isReadyToShoot();
    var isInTurretRange = TurretSubsystem.isYawInShootingRange(turretYawTarget);

    // Get driver translation speeds
    chassisSpeeds.vxMetersPerSecond = xSupplier.get().in(MetersPerSecond) * 0.35;
    chassisSpeeds.vyMetersPerSecond = ySupplier.get().in(MetersPerSecond) * 0.35;

    // Aim drivetrain
    // NOTE: Slew rate limit needs to be applied so the robot slows properly (see 2022 robot doing "stoppies")
    if (isInTurretRange) {
        // Prepare shooter
      shooter.prepareToShoot(shootingSettings.getVelocity());
    
      // Set the turret position
      turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
      turretSubsystem.moveToShootingYawPosition(turretYawTarget);

      // Turret can reach, stop robot

      var limitedChassisSpeeds = rateLimiter.calculate(chassisSpeeds);
      drivetrain.setControl(swerveRequestRotation
          .withVelocityX(limitedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(limitedChassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(0.0));

    } else {
      // Turret cannot reach, turn robot
      var limitedChassisSpeeds = rateLimiter.calculate(chassisSpeeds);

      // Decide the direction to turn, then set the robot rotation target so the turret's shooting yaw limit on that
      // side is pointed at the target
      Rotation2d robotTargetDirection = angleToTarget.minus(fromRadians(PI)); // Turret is on the back of the robot
      if (robotTargetDirection.minus(robotPose.getRotation()).getRadians() > 0) {
        robotTargetDirection = robotTargetDirection.minus(DRIVETRAIN_YAW_LIMIT_FORWARD);
      } else {
        robotTargetDirection = robotTargetDirection.minus(DRIVETRAIN_YAW_LIMIT_REVERSE);
      }

      // If the drivetrain is getting close, start getting ready to shoot
      if (Math.abs(robotPose.getRotation().minus(robotTargetDirection).getDegrees()) < 25) {
        // Prepare the shooter
        shooter.prepareToShoot(shootingSettings.getVelocity());
        
        // Set the turret position
        turretSubsystem.moveToShootingYawPosition(turretYawTarget);
        turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
      }

      drivetrain.setControl(swerveRequestFacing
          .withVelocityX(limitedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(limitedChassisSpeeds.vyMetersPerSecond)
          .withTargetDirection(robotTargetDirection));
    }

    var isPitchReady = turretSubsystem.isAtPitchTarget();
    var isYawReady = turretSubsystem.isAtYawTarget();
    if (isShooterReady && isPitchReady && isYawReady) {
      // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot and start timer
      turretSubsystem.shoot();
      isShooting = true;
    }

    // Update LEDs with ready state
    if (isShooting) {
      ledSubsystem.setUpdater(l -> l.setAll(kGreen));
    } else {
      ledSubsystem.setUpdater(l -> 
          l.setLEDSegments(kBlue, isShooterReady, isInTurretRange, isPitchReady, isYawReady));
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    turretSubsystem.prepareToIntake();
    drivetrain.setControl(new SwerveRequest.Idle());
    ledSubsystem.setUpdater(null);
  }

}
