package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to intake a note from the floor into the amper
 */
public class AutoIntakeToAmperCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private boolean hasSeenNote = false;

  private final MutableMeasure<Velocity<Angle>> omega = MutableMeasure.zero(RadiansPerSecond);
  private final MutableMeasure<Velocity<Distance>> distance = MutableMeasure.zero(MetersPerSecond);

  private final Translation2d translation2d = new Translation2d();
  private final LimelightTarget_Retro LimelightTarget_Retro = new LimelightTarget_Retro();

private final ChassisSpeedsRateLimiter rateLimiter = new ChassisSpeedsRateLimiter(
      TRANSLATION_RATE_LIMIT.in(MetersPerSecondPerSecond), ROTATION_RATE_LIMIT.in(RadiansPerSecond.per(Second)));

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_VELOCITY.times(DEADBAND).in(MetersPerSecond))
      .withRotationalDeadband(MAX_ANGULAR_VELOCITY.times(DEADBAND).in(RadiansPerSecond))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic);

  // Desired chassis speeds. Defined here to prevent reallocation.
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();


  public AutoIntakeToAmperCommand(
  IntakeSubsystem intakeSubsystem,  
  AmperSubsystem amperSubsystem,
  CommandSwerveDrivetrain drivetrainSubsystem,
  LimelightHelpers limelightHelpers) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(intakeSubsystem, amperSubsystem, drivetrainSubsystem);
  }

  public Supplier<Measure<Velocity<Angle>>> getTargetRotation() {
    double rotationAngle = LimelightTarget_Retro.getTargetPose_CameraSpace2D().getRotation().getRadians();
        return () -> omega.mut_replace(rotationAngle, RadiansPerSecond);
  }

  public Supplier<Measure<Velocity<Distance>>> getTargetDistance() {
    double distanceToTarget = translation2d.getDistance(LimelightTarget_Retro.getTargetPose_CameraSpace2D().getTranslation());
      return () -> distance.mut_replace(distanceToTarget, MetersPerSecond);
  }

  @Override 
  public void initialize() {
    // Reset the slew rate limiters, in case the robot is already moving
    rateLimiter.reset(drivetrainSubsystem.getCurrentFieldChassisSpeeds());
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight") || hasSeenNote) {
      desiredChassisSpeeds.vxMetersPerSecond = getTargetDistance().get().in(MetersPerSecond);
      desiredChassisSpeeds.vyMetersPerSecond = getTargetDistance().get().in(MetersPerSecond);
      desiredChassisSpeeds.omegaRadiansPerSecond = getTargetRotation().get().in(RadiansPerSecond);
      var limitedCassisSpeeds2 = rateLimiter.calculate(desiredChassisSpeeds);
      drivetrainSubsystem.setControl(drive
        .withVelocityX(limitedCassisSpeeds2.vxMetersPerSecond)
        .withVelocityY(limitedCassisSpeeds2.vyMetersPerSecond)
        .withRotationalRate(limitedCassisSpeeds2.omegaRadiansPerSecond));
    }
    intakeSubsystem.intake();
    amperSubsystem.load();
  }

  @Override
  public boolean isFinished() {
    return amperSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    amperSubsystem.stop();
  }

}
