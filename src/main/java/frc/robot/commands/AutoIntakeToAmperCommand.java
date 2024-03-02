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

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
  private double speed;
  private double rotation;

  private final PIDController rotationPidController = new PIDController(0, 0, 0);
  private final PIDController speedPidController = new PIDController(0, 0, 0);

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
      CommandSwerveDrivetrain drivetrainSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    
    addRequirements(intakeSubsystem, amperSubsystem, drivetrainSubsystem);
  }
  
  public void driveToTarget(double speed, double rotation) {
    var xSpeed = speed;
    var zRotation = rotation;
    desiredChassisSpeeds.vxMetersPerSecond = xSpeed;
    desiredChassisSpeeds.vyMetersPerSecond = 0;
    desiredChassisSpeeds.omegaRadiansPerSecond = zRotation;
    var limitedCassisSpeeds = rateLimiter.calculate(desiredChassisSpeeds);
    drivetrainSubsystem.setControl(drive
        .withVelocityX(limitedCassisSpeeds.vxMetersPerSecond)
        .withVelocityY(limitedCassisSpeeds.vyMetersPerSecond)
        .withRotationalRate(limitedCassisSpeeds.omegaRadiansPerSecond));
  }

  @Override 
  public void initialize() {
    // Reset the slew rate limiters, in case the robot is already moving
    rateLimiter.reset(drivetrainSubsystem.getCurrentFieldChassisSpeeds());
  }

  @Override
  public void execute() {
    var results = LimelightHelpers.getLatestResults("limelight");
    if (results.targetingResults.valid && results.targetingResults.targets_Detector.length>0) {
      rotation = -rotationPidController.calculate(results.targetingResults.targets_Detector[0].ty);
      speed = -speedPidController.calculate(results.targetingResults.targets_Detector[0].ty);
      hasSeenNote = true;
      driveToTarget(speed, rotation);
    } else if (hasSeenNote) {
      driveToTarget(speed, rotation);
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
