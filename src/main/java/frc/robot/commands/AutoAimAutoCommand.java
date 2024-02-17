package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.team;
import static frc.robot.Constants.TurretConstants.TURRET_ANGLE_THRESHOLD;
import static frc.robot.Constants.TurretConstants.TURRET_MAXIMUM_READYSHOOT_DISTANCE;
import static frc.robot.Constants.TurretConstants.speakerPosition1;
import static frc.robot.Constants.TurretConstants.speakerPosition2;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimAutoCommand extends Command {
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private CommandSwerveDrivetrain drivetrainSubsystem;

  private final Translation2d speakerPosition = team ? speakerPosition1 : speakerPosition2;

  private Pose2d currentDrivetrainPose;
  private Measure<Distance> currentSpeakerDistance;
  private Rotation2d turretAngle;
  private Rotation2d drivetrainAngle;
  private Rotation2d targetAngle;

  private Boolean isTurretReady;
  private Boolean isDrivetrainReady;


  public AutoAimAutoCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrainSubsystem) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(turretSubsystem, shooterSubsystem, drivetrainSubsystem);
  }

  private Boolean update() {
    currentDrivetrainPose = drivetrainSubsystem.getState().Pose;
    currentSpeakerDistance =  Meters.of(currentDrivetrainPose.getTranslation().getDistance(speakerPosition));
    drivetrainAngle = currentDrivetrainPose.getRotation();
    turretAngle = turretSubsystem.getCurrentAngle(drivetrainAngle);
    targetAngle = currentDrivetrainPose.getTranslation().minus(speakerPosition).getAngle().rotateBy(fromRadians(PI));

    isTurretReady = abs(turretAngle.minus(targetAngle).getDegrees()) <= TURRET_ANGLE_THRESHOLD;
    isDrivetrainReady = currentSpeakerDistance.lte(TURRET_MAXIMUM_READYSHOOT_DISTANCE);

    return isTurretReady && isDrivetrainReady && shooterSubsystem.isShooterReady();
  }

  @Override
  public void execute() {
    update();
    shooterSubsystem.spinShooterWheel();

    if (!isTurretReady) {
      turretSubsystem.toAngle(targetAngle);
    }
  }

  @Override
  public boolean isFinished() {
    // update returns if the turret is ready for shooting: the turret is correctly angled, the drivetrain is correctly angled, the robot is within the maximum distance to the speaker, and the shooter wheel is ready
    return update();
  }

@Override
public void end(boolean interrupted) {
  turretSubsystem.stop();
  shooterSubsystem.stop();
}
}
