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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
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
  private boolean hasSeenNote;
  private double rotation;

  private final PIDController rotationPidController = new PIDController(.1, 0, 0);

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
      Supplier<Measure<Velocity<Distance>>> translationXSupplier,
      Supplier<Measure<Velocity<Distance>>> translationYSupplier,
      Supplier<Measure<Velocity<Angle>>> rotationSupplier) {
    
    this.intakeSubsystem = intakeSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(intakeSubsystem, turretSubsystem, amperSubsystem, drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
    
    // Reset the slew rate limiters, in case the robot is already moving
    rateLimiter.reset(drivetrainSubsystem.getCurrentFieldChassisSpeeds());
    
    hasSeenNote = false;
    rotationPidController.reset();
  }

  @Override
  public void execute() {
    var results = LimelightHelpers.getLatestResults("limelight");
    if (results.targetingResults.valid && results.targetingResults.targets_Detector.length > 0) {
      rotation = rotationPidController.calculate(results.targetingResults.targets_Detector[0].tx);
      hasSeenNote = true;
      desiredChassisSpeeds.omegaRadiansPerSecond = rotation;
    } else if (hasSeenNote) {
      desiredChassisSpeeds.omegaRadiansPerSecond = rotation;
    } else {
      desiredChassisSpeeds.omegaRadiansPerSecond = rotationSupplier.get().in(RadiansPerSecond);
    }

    desiredChassisSpeeds.vxMetersPerSecond = translationXSupplier.get().in(MetersPerSecond);
    desiredChassisSpeeds.vyMetersPerSecond = translationYSupplier.get().in(MetersPerSecond);
    var limitedCassisSpeeds = rateLimiter.calculate(desiredChassisSpeeds);

    SmartDashboard.putNumber("Rotation", rotation);

    drivetrainSubsystem.setControl(drive
        .withVelocityX(limitedCassisSpeeds.vxMetersPerSecond)
        .withVelocityY(limitedCassisSpeeds.vyMetersPerSecond)
        .withRotationalRate(limitedCassisSpeeds.omegaRadiansPerSecond));

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
