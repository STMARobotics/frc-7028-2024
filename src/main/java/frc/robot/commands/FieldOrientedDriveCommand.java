package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.ROTATIONAL_DEADBAND;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_DEADBAND;
import static frc.robot.Constants.TeleopDriveConstants.TRANSLATION_RATE_LIMIT;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.ChassisSpeedsRateLimiter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Command for teleop driving where translation is field oriented and rotation velocity is
 * controlled by the driver.
 *
 * <p>
 * Translation is specified on the field-relative coordinate system. The Y-axis runs parallel to
 * the alliance wall, left is positive. The X-axis runs down field toward the opposing alliance
 * wall, away from the alliance wall is positive.
 */
public class FieldOrientedDriveCommand extends Command {
	private final CommandSwerveDrivetrain drivetrainSubsystem;
	private final Supplier<LinearVelocity> translationXSupplier;
	private final Supplier<LinearVelocity> translationYSupplier;
	private final Supplier<AngularVelocity> rotationSupplier;

	private final ChassisSpeedsRateLimiter rateLimiter = new ChassisSpeedsRateLimiter(
			TRANSLATION_RATE_LIMIT.in(MetersPerSecondPerSecond),
			ROTATION_RATE_LIMIT.in(RadiansPerSecond.per(Second)));

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_VELOCITY.times(TRANSLATION_DEADBAND).in(MetersPerSecond))
			.withRotationalDeadband(
					MAX_ANGULAR_VELOCITY.times(ROTATIONAL_DEADBAND).in(RadiansPerSecond))
			.withDriveRequestType(DriveRequestType.Velocity)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	// Desired chassis speeds. Defined here to prevent reallocation.
	private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

	/**
	 * Constructor
	 *
	 * @param drivetrainSubsystem drivetrain
	 * @param translationXSupplier supplier for translation X component, in meters per second
	 * @param translationYSupplier supplier for translation Y component, in meters per second
	 * @param rotationSupplier supplier for rotation component, in radians per second
	 */
	public FieldOrientedDriveCommand(
			CommandSwerveDrivetrain drivetrainSubsystem,
			Supplier<LinearVelocity> translationXSupplier,
			Supplier<LinearVelocity> translationYSupplier,
			Supplier<AngularVelocity> rotationSupplier) {
		this.drivetrainSubsystem = drivetrainSubsystem;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		addRequirements(drivetrainSubsystem);
	}

	@Override
	public void initialize() {
		// Reset the slew rate limiters, in case the robot is already moving
		rateLimiter.reset(drivetrainSubsystem.getCurrentFieldChassisSpeeds());
	}

	@Override
	public void execute() {
		desiredChassisSpeeds.vxMetersPerSecond = translationXSupplier.get().in(MetersPerSecond);
		desiredChassisSpeeds.vyMetersPerSecond = translationYSupplier.get().in(MetersPerSecond);
		desiredChassisSpeeds.omegaRadiansPerSecond = rotationSupplier.get().in(RadiansPerSecond);
		var limitedCassisSpeeds = rateLimiter.calculate(desiredChassisSpeeds);
		drivetrainSubsystem.setControl(
				drive
						.withVelocityX(limitedCassisSpeeds.vxMetersPerSecond)
						.withVelocityY(limitedCassisSpeeds.vyMetersPerSecond)
						.withRotationalRate(limitedCassisSpeeds.omegaRadiansPerSecond));
	}

	@Override
	public void end(boolean interrupted) {
		drivetrainSubsystem.setControl(new SwerveRequest.Idle());
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
