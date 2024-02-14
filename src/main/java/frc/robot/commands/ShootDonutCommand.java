package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.AutoDriveConstants.THETA_kD;
import static frc.robot.Constants.AutoDriveConstants.THETA_kI;
import static frc.robot.Constants.AutoDriveConstants.THETA_kP;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to run shoot donut
 */
public class ShootDonutCommand extends Command {

  private static final double SHOOT_TIME = 5;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);


  private final Timer shootTimer = new Timer();

  public ShootDonutCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, 
  CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(indexerSubsystem, shooterSubsystem, drivetrain);

    swerveRequest.ForwardReference = ForwardReference.RedAlliance;
    swerveRequest.HeadingController = new PhoenixPIDController(THETA_kP, THETA_kI, THETA_kD);
  }

  @Override
  public void initialize() {
    swerveRequest.HeadingController.reset();
    shootTimer.reset();
  }

  @Override
  public void execute() {
    shooterSubsystem.setAimPosition(Radians.of(Math.PI));
    shooterSubsystem.spinShooterWheel(40);
    if (shooterSubsystem.isShooterReady())
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