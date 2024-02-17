package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TeleopAutoAim extends Command {

  // To be determined and may be moved to another file
  private final Translation2d speakerPosition = new Translation2d(123, 123); 

  private CommandSwerveDrivetrain drivetrain;
  private TurretSubsystem turret;
  private ShooterSubsystem shooter;
  private Boolean preparingShooterWheel = false;
  private SwerveDriveState currentDrivetrainState = drivetrain.getState();
  private Translation2d currentPosition;
  private Rotation2d targetAngle;
  private Rotation2d turretAngle;
  private Rotation2d drivetrainAngle;
  private SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);


  private void update() {
    currentDrivetrainState = drivetrain.getState();
    currentPosition = currentDrivetrainState.Pose.getTranslation();
    drivetrainAngle = currentPosition.getAngle() ;
    turretAngle = turret.getCurrentAngle();
  }

  private boolean robotIsFacingSpeaker() {
    return (abs(targetAngle.getDegrees() - drivetrainAngle.getDegrees()) <= 5) && abs(targetAngle.getDegrees() - turretAngle.getDegrees()) <= 5;
  }

  public TeleopAutoAim(CommandSwerveDrivetrain drivetrainSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
    drivetrain = drivetrainSubsystem;
    turret = turretSubsystem;
    shooter = shooterSubsystem;
    addRequirements(drivetrain, turret, shooter);
  }

  @Override
  public void initialize() {
    update();
    preparingShooterWheel = false;
  }

  @Override
  public void execute() {
    update();

    if (robotIsFacingSpeaker()) {
      shooter.spinShooterWheel();
      preparingShooterWheel = true;
    } else {
      var targetAngle = currentPosition.minus(speakerPosition).getAngle().rotateBy(fromRadians(PI));
      turret.toAngle(targetAngle);
      drivetrain.setControl(swerveRequest.withTargetDirection(targetAngle));
    }
  }

  @Override
  public boolean isFinished() {
    return preparingShooterWheel;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    shooter.stop();
    // Will add drivetrain logic later 
  }
}
