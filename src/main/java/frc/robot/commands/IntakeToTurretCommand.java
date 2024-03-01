package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_RATE_LIMIT;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.LimelightCalcs;
import frc.robot.limelight.LimelightDetectorTarget;
import frc.robot.limelight.LimelightResults;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Command to intake a note from the floor to the turret
 */
public class IntakeToTurretCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Measure<Velocity<Distance>>> translationXSupplier;
  private final Supplier<Measure<Velocity<Distance>>> translationYSupplier;
  private final Supplier<Measure<Velocity<Angle>>> rotationSupplier;
  private final LimelightResults limelightResults;
  private final LimelightCalcs limelightCalcs; 
  private final LimelightDetectorTarget limelightDetectorTarget;
  private boolean hasSeenNote = false;

  private final ChassisSpeedsRateLimiter rateLimiter = new ChassisSpeedsRateLimiter(
      TRANSLATION_RATE_LIMIT.in(MetersPerSecondPerSecond), ROTATION_RATE_LIMIT.in(RadiansPerSecond.per(Second)));

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_VELOCITY.times(DEADBAND).in(MetersPerSecond))
      .withRotationalDeadband(MAX_ANGULAR_VELOCITY.times(DEADBAND).in(RadiansPerSecond))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic);

  // Desired chassis speeds. Defined here to prevent reallocation.
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  public IntakeToTurretCommand(
  IntakeSubsystem intakeSubsystem, 
  TurretSubsystem turretSubsystem, 
  AmperSubsystem amperSubsystem,
  CommandSwerveDrivetrain drivetrainSubsystem,
  LimelightSubsystem limelightSubsystem, 
  Supplier<Measure<Velocity<Distance>>> translationXSupplier,
  Supplier<Measure<Velocity<Distance>>> translationYSupplier,
  Supplier<Measure<Velocity<Angle>>> rotationSupplier,
  LimelightResults limelightResults,
  LimelightCalcs limelightCalcs,
  LimelightDetectorTarget limelightDetectorTarget) {
    this.intakeSubsystem = intakeSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.limelightResults = limelightResults;
    this.limelightCalcs = limelightCalcs;
    this.limelightDetectorTarget = limelightDetectorTarget;

    addRequirements(intakeSubsystem, turretSubsystem, amperSubsystem, drivetrainSubsystem);
  }

  public Rotation2d getTargetRotation() {
     return limelightCalcs.getRobotRelativeTargetInfo(
        limelightDetectorTarget.targetXDegrees, limelightDetectorTarget.targetYDegrees).angle;
        
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
    
    // Reset the slew rate limiters, in case the robot is already moving
    rateLimiter.reset(drivetrainSubsystem.getCurrentFieldChassisSpeeds());
  }
  
  @Override
  public void execute() {
    desiredChassisSpeeds.vxMetersPerSecond = translationXSupplier.get().in(MetersPerSecond);
    desiredChassisSpeeds.vyMetersPerSecond = translationYSupplier.get().in(MetersPerSecond);
    desiredChassisSpeeds.omegaRadiansPerSecond = rotationSupplier.get().in(RadiansPerSecond);
    var limitedCassisSpeeds = rateLimiter.calculate(desiredChassisSpeeds);
    drivetrainSubsystem.setControl(drive
        .withVelocityX(limitedCassisSpeeds.vxMetersPerSecond)
        .withVelocityY(limitedCassisSpeeds.vyMetersPerSecond)
        .withRotationalRate(limitedCassisSpeeds.omegaRadiansPerSecond));
    if (limelightResults.valid || hasSeenNote) {
      hasSeenNote = true;
      desiredChassisSpeeds.vxMetersPerSecond = translationXSupplier.get().in(MetersPerSecond);
      desiredChassisSpeeds.vyMetersPerSecond = translationYSupplier.get().in(MetersPerSecond);
      desiredChassisSpeeds.omegaRadiansPerSecond = getTargetRotation().getRadians();
      var limitedCassisSpeeds2 = rateLimiter.calculate(desiredChassisSpeeds);
      drivetrainSubsystem.setControl(drive
        .withVelocityX(limitedCassisSpeeds2.vxMetersPerSecond)
        .withVelocityY(limitedCassisSpeeds2.vyMetersPerSecond)
        .withRotationalRate(limitedCassisSpeeds2.omegaRadiansPerSecond));
    }
    if (turretSubsystem.isAtYawAndPitchTarget()) {
      intakeSubsystem.intake();
      amperSubsystem.intake();
      turretSubsystem.load();
    }
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    turretSubsystem.stopPitch();
    turretSubsystem.stopYaw();
    turretSubsystem.runRollers(RotationsPerSecond.zero());
    amperSubsystem.stop();
  }

}
