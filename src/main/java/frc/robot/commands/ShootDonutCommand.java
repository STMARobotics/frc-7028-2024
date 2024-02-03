package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.VelocityAngleInterpolator;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shooter motors
 */
public class ShootDonutCommand extends Command {

  private static final double SHOOT_TIME = 0.25;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final VelocityAngleInterpolator VelocityAngleInterpolator;
  private final Supplier<Pose2d> robotSupplier;

  private final Timer shootTimer = new Timer();
  protected double shooterRPS = 1;
  private double shooterSpeedGoal;
  private double radiansToRotate;
  private double distance;

  /**
   * Constructor
   * 
   * @param shooterRPS       velocity of shooter in rotations per second
   * @param shooterSubsystem shooter
   */
  public ShootDonutCommand(VelocityAngleInterpolator velocityAngleInterpolator, ShooterSubsystem shooterSubsystem,
      IndexerSubsystem indexerSubsystem, Supplier<Pose2d> robotPoseSupplier) {
    this.VelocityAngleInterpolator = velocityAngleInterpolator;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.robotSupplier = robotPoseSupplier;

    addRequirements(indexerSubsystem, shooterSubsystem);
  }

  public double findDistance() {
    var targetTranslation = new Translation2d(0.5, 2);
    double distanceToTarget = robotPoseSupplier.getTranslation().getDistance(targetTranslation);
    return distanceToTarget;
  }

  @Override
  public void initialize() {
    shootTimer.reset();
  }

  @Override
  public void execute() {
    shooterSubsystem.actuatorRotate(radiansToRotate);
    shooterSubsystem.spinShooterWheel(VelocityAngleInterpolator.calculate(findDistance()));
    if (shooterSubsystem.checkShooterSpeed(shooterSpeedGoal) &&
        shooterSubsystem.checkWristPosition(radiansToRotate)) {
      indexerSubsystem.fireDonut(Volts.of(3));
      shootTimer.start();
    }
  }

  @Override
  public boolean isFinished() {
    return (shootTimer.hasElapsed(SHOOT_TIME));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.actuatorStop();
    shooterSubsystem.stop();
    shootTimer.stop();
  }
}