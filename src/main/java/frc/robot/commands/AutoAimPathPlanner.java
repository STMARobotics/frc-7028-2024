package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static frc.robot.Constants.TurretConstants.TURRET_ANGLE_THRESHOLD;
import static frc.robot.Constants.TurretConstants.relativeSpeakerPosition;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimPathPlanner extends Command {
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private CommandSwerveDrivetrain drivetrainSubsystem;


  private Pose2d currentDrivetrainPose;
  private Rotation2d turretAngle;

  private Boolean preparingShooterWheel;

  public AutoAimPathPlanner(TurretSubsystem turretSubsystem2, ShooterSubsystem shooterSubsystem2, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.turretSubsystem = turretSubsystem2;
    this.shooterSubsystem = shooterSubsystem2;
    this.drivetrainSubsystem = commandSwerveDrivetrain;

    addRequirements(turretSubsystem2, shooterSubsystem2, commandSwerveDrivetrain);
  }

  private void update() {
    currentDrivetrainPose = drivetrainSubsystem.getState().Pose;
    turretAngle = turretSubsystem.getCurrentAngle();
  }

  @Override
  public void initialize() {
    preparingShooterWheel = false;
    update();
  }

  @Override
  public void execute() {
    update();
    Rotation2d targetAngle = currentDrivetrainPose.getTranslation().minus(relativeSpeakerPosition).getAngle().rotateBy(fromRadians(PI));

    if (abs(turretAngle.getDegrees() - targetAngle.getDegrees()) > TURRET_ANGLE_THRESHOLD) {
      turretSubsystem.toAngle(targetAngle);
    } else {
      shooterSubsystem.spinShooterWheel();
      preparingShooterWheel = true;
    }
  }

  @Override
  public boolean isFinished() {
    return preparingShooterWheel;
  }

@Override
public void end(boolean interrupted) {
  turretSubsystem.stop();
  shooterSubsystem.stop();
}
}
