package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shooter motors
 */
public class ShootDonutCommand extends Command {

  private static final double SHOOT_TIME = 5;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  // private final VelocityAngleInterpolator velocityAngleInterpolator;
  // private final Supplier<Pose2d> robotPoseSupplier;

  private final Timer shootTimer = new Timer();

  /**
   * Constructor
   * 
   * @param shooterRPS       velocity of shooter in rotations per second
   * @param shooterSubsystem shooter
   */
  public ShootDonutCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    // this.velocityAngleInterpolator = velocityAngleInterpolator;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    // this.robotPoseSupplier = robotPoseSupplier;

    addRequirements(indexerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    shootTimer.reset();
  }

  @Override
  public void execute() {
    // var targetTranslation = new Translation2d(0.5, 2);
    // double distanceToTarget =
    // robotPoseSupplier.get().getTranslation().getDistance(targetTranslation);
    // var donutShooterSettings =
    // velocityAngleInterpolator.calculate(distanceToTarget);
    // shooterSubsystem.actuatorRotate(donutShooterSettings.angle);
    shooterSubsystem.spinShooterWheel(40);
    if (shooterSubsystem.checkShooterSpeed(40))
      indexerSubsystem.shoot();
      shootTimer.start();
  }

  @Override
  public boolean isFinished() {
    return (shootTimer.hasElapsed(SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.actuatorStop();
    indexerSubsystem.stop();
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}