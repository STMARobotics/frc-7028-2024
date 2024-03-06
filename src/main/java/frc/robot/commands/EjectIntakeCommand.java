package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Runs everything in reverse to attempt to eject a jammed note
 */
public class EjectIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final AmperSubsystem amperSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final CommandSwerveDrivetrain drivetrainSubsystem;

  private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(-0.5);

  public EjectIntakeCommand(IntakeSubsystem intakeSubsystem, AmperSubsystem amperSubsystem,
      TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrainSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.amperSubsystem = amperSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(intakeSubsystem, amperSubsystem, turretSubsystem, shooterSubsystem, drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    turretSubsystem.prepareToExchange();
  }

  @Override
  public void execute() {
    drivetrainSubsystem.setControl(swerveRequest);
    if (turretSubsystem.isInExchangePosition()) {
      amperSubsystem.load();
      intakeSubsystem.reverse();
      turretSubsystem.eject();
      shooterSubsystem.reverse();
    }
  }

  @Override
  public void end(boolean interrupted) {
    amperSubsystem.stop();
    intakeSubsystem.stop();
    turretSubsystem.stop();
    shooterSubsystem.stop();
    drivetrainSubsystem.setControl(new SwerveRequest.Idle());
  }
  
}
