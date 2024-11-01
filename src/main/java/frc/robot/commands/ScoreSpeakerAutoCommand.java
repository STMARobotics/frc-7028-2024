package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.SHOOT_WHILE_MOVING_COEFFICIENT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * This command automatically scores in the speaker while the robot is moving. It does not do
 * anything with the drivetrain, only shooting when the turret can reach.
 */
public class ScoreSpeakerAutoCommand extends Command {

  // private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Supplier<ChassisSpeeds> fieldRelativeSpeedSupplier;
  private final Supplier<Pose2d> poseSupplier;

  // Reusable objects to prevent reallocation (to reduce memory pressure)
  private final MutAngle turretYawTarget = Rotations.mutable(0);

  private final Translation2d targetRed;
  private final Translation2d targetBlue;
  private Translation2d speakerTranslation;

  private boolean isShooting = false;

  public ScoreSpeakerAutoCommand(
      ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<ChassisSpeeds> fieldRelativeSpeedSupplier,
      Supplier<Pose2d> poseSupplier,
      Translation2d targetRed,
      Translation2d targetBlue) {
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.fieldRelativeSpeedSupplier = fieldRelativeSpeedSupplier;
    this.poseSupplier = poseSupplier;
    this.targetRed = targetRed;
    this.targetBlue = targetBlue;

    addRequirements(shooter, turretSubsystem);
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    speakerTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? targetBlue : targetRed;
    isShooting = false;
  }

  @Override
  public void execute() {
    var robotPose = poseSupplier.get();

    // Translation to the center of the turret
    var turretTranslation = TurretSubsystem.getTurretTranslation(robotPose);

    // Distance between the robot and the speaker
    var distanceToSpeaker = turretTranslation.getDistance(speakerTranslation);

    // Lookup shooter settings for this distance
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker);

    // Calculate time to hit speaker
    var timeUntilScored = SHOOT_WHILE_MOVING_COEFFICIENT
        * (distanceToSpeaker / shootingSettings.getVelocity().in(RotationsPerSecond));

    // Calculate the predicted offset of the speaker compared to current pose (in meters)
    var fieldRelativeSpeed = fieldRelativeSpeedSupplier.get();
    var speakerPredictedOffset = new Translation2d(
        fieldRelativeSpeed.vxMetersPerSecond * timeUntilScored,
        fieldRelativeSpeed.vyMetersPerSecond * timeUntilScored);

    var predictedSpeakerTranslation = speakerTranslation.minus(speakerPredictedOffset);

    var predictedDist = predictedSpeakerTranslation.getDistance(turretTranslation);

    // Calculate the angle to the speaker
    var angleToSpeaker = predictedSpeakerTranslation.minus(turretTranslation).getAngle();

    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(
        angleToSpeaker.minus(robotPose.getRotation()).getRotations(), Rotations);

    shootingSettings = SHOOTER_INTERPOLATOR.calculate(predictedDist);

    // Calculate ready state
    var isShooterReady = shooter.isReadyToShoot();

    // Set the turret position
    turretSubsystem.moveToShootingYawPosition(turretYawTarget);
    turretSubsystem.moveToPitchPosition(shootingSettings.getPitch());
    shooter.prepareToShoot(shootingSettings.getVelocity());

    var isPitchReady = turretSubsystem.isAtPitchTarget();
    var isYawReady = turretSubsystem.isAtYawTarget();
    if (isShooterReady
        && isPitchReady
        && isYawReady
        && TurretSubsystem.isYawInShootingRange(turretYawTarget)) {
      // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot
      // and start timer
      turretSubsystem.shoot();
      isShooting = true;
    }

    // Update LEDs with ready state
    if (isShooting) {
      ledSubsystem.setUpdater(l -> l.setAll(kGreen));
    } else {
      ledSubsystem.setUpdater(
          l -> l.setLEDSegments(kBlue, isShooterReady, isPitchReady, isYawReady));
    }
  }

  @Override
  public boolean isFinished() {
    return !turretSubsystem.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    turretSubsystem.prepareToIntake();
    ledSubsystem.setUpdater(null);
    turretSubsystem.stopRollers();
  }
}
