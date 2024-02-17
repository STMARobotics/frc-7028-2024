package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimShootCommand extends Command {

  // To be determined and may be moved to another file
  private final Translation2d speakerPosition = new Translation2d(123, 123); 

  private CommandSwerveDrivetrain drivetrain;
  private TurretSubsystem turret;
  private ShooterSubsystem shooter;
  private Boolean hasShot = false;
  private SwerveDriveState currentDrivetrainState = drivetrain.getState();
  private Translation2d currentPosition;
  private Double targetAngle;
  private Double turretAngle;
  private Double drivetrainAngle;
  private SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);


  private void update() {
    currentDrivetrainState = drivetrain.getState();
    currentPosition = currentDrivetrainState.Pose.getTranslation();
    drivetrainAngle = currentPosition.getAngle().getDegrees();
    turretAngle = turret.getCurrentAngle();
  }

  private boolean robotIsFacingSpeaker() {
    return (abs(targetAngle - drivetrainAngle) <= 5) && abs(targetAngle - turretAngle) <= 5;
  }

  public AutoAimShootCommand(CommandSwerveDrivetrain drivetrainSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
    drivetrain = drivetrainSubsystem;
    turret = turretSubsystem;
    shooter = shooterSubsystem;
    addRequirements(drivetrain, turret, shooter);
  }

  @Override
  public void initialize() {
    update();
    hasShot = false;
  }

  @Override
  public void execute() {
    update();
    if (robotIsFacingSpeaker()) {
      shooter.spinShooterWheel();
      hasShot = true;
    } else {
      var targetAngle = currentPosition.minus(speakerPosition).getAngle().rotateBy(fromRadians(PI));
      turret.toAngle(targetAngle);
      drivetrain.setControl(swerveRequest.withTargetDirection(targetAngle));
    }
  }

  @Override
  public boolean isFinished() {
    return robotIsFacingSpeaker() && hasShot;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    // Will add drivetrain logic later 
  }
}
