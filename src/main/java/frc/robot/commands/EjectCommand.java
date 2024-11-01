package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/** Runs everything in reverse to attempt to eject a jammed note */
public class EjectCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private final TurretSubsystem turretSubsystem;
	private final ShooterSubsystem shooterSubsystem;
	private final CommandSwerveDrivetrain drivetrainSubsystem;

	private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.Velocity)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo)
			.withVelocityX(-0.5);

	public EjectCommand(
			IntakeSubsystem intakeSubsystem,
			TurretSubsystem turretSubsystem,
			ShooterSubsystem shooterSubsystem,
			CommandSwerveDrivetrain drivetrainSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.turretSubsystem = turretSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.drivetrainSubsystem = drivetrainSubsystem;

		addRequirements(intakeSubsystem, turretSubsystem, shooterSubsystem, drivetrainSubsystem);
	}

	@Override
	public void initialize() {
		turretSubsystem.prepareToIntake();
	}

	@Override
	public void execute() {
		drivetrainSubsystem.setControl(swerveRequest);
		if (turretSubsystem.isInIntakePosition()) {
			intakeSubsystem.reverse();
			turretSubsystem.eject();
			shooterSubsystem.reverse();
		}
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.stop();
		turretSubsystem.stop();
		shooterSubsystem.stop();
		drivetrainSubsystem.setControl(new SwerveRequest.Idle());
	}
}
