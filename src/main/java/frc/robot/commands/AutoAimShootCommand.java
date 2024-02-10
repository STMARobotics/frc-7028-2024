package frc.robot.commands;

import static java.lang.Math.abs;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private Pose2d currentDrivetrainPose  = currentDrivetrainState.Pose;
  private Translation2d deltaDrivetrainPosition;
  private Rotation2d deltaDrivetrainAngle;
  private Rotation2d deltaTurretAngle;

  private void update() {
    currentDrivetrainState = drivetrain.getState();
    currentDrivetrainPose  = currentDrivetrainState.Pose;
    deltaDrivetrainPosition = currentDrivetrainPose.getTranslation().minus(speakerPosition);
    deltaDrivetrainAngle = deltaDrivetrainPosition.getAngle();
  }

  private boolean isFacingSpeaker() {
    return (abs(deltaDrivetrainAngle.getDegrees()) <= 5) && (abs(deltaTurretAngle.getDegrees()) <= 5);
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

    // I will add rest of logic later; this is just the base logic for autoaiming 

    if (isFacingSpeaker()) {
      shooter.shootDonut();
      hasShot = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFacingSpeaker() && hasShot;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    // Will add drivetrain logic later 
  }
}
