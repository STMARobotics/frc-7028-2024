package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LEDConstants.BABYBIRD_COLOR;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_MID_BLUE;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_MID_RED;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.commands.BabyBirdCommand;
import frc.robot.commands.BloopCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreSpeakerAutoCommand;
import frc.robot.commands.ShootTeleopCommand;
import frc.robot.commands.led.LEDAlternateCommand;
import frc.robot.commands.led.LEDBlinkCommand;
import frc.robot.commands.led.LEDMarqueeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Non-static command factory for creating commands. See
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories
 */
public class AutoCommands {

	private final CommandSwerveDrivetrain drivetrainSubsystem;
	private final ShooterSubsystem shooterSubsystem;
	private final TurretSubsystem turretSubsystem;
	private final IntakeSubsystem intakeSubsystem;
	private final LEDSubsystem ledSubsystem;

	public AutoCommands(
			CommandSwerveDrivetrain drivetrainSubsystem,
			ShooterSubsystem shooterSubsystem,
			TurretSubsystem turretSubsystem,
			IntakeSubsystem intakeSubsystem,
			LEDSubsystem ledSubsystem) {
		this.drivetrainSubsystem = drivetrainSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.turretSubsystem = turretSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.ledSubsystem = ledSubsystem;
	}

	/** Registers named commands for PathPlanner */
	public void registerPPNamedCommands() {
		NamedCommands.registerCommand("intake", intakeToTurret());
		NamedCommands.registerCommand("scoreSpeakerWhileMoving", autoScoreSpeaker());
		NamedCommands.registerCommand("firstAutoShot", autoScoreScoreSpeakerFarSideFirstShot());
		NamedCommands.registerCommand("bloopNorth", bloopFirstNorth());
		NamedCommands.registerCommand("bloopSouth", bloopFirstSouth());
		NamedCommands.registerCommand("bloopEast", bloopFirstEast());
		NamedCommands.registerCommand("bloopWest", bloopFirstWest());
	}

	/**
	 * Command to automatically score in the speaker while moving
	 *
	 * @return new command
	 */
	public Command autoScoreSpeaker() {
		return new ScoreSpeakerAutoCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				drivetrainSubsystem::getCurrentFieldChassisSpeeds,
				() -> drivetrainSubsystem.getState().Pose,
				ShootingConstants.SPEAKER_RED_AUTO,
				ShootingConstants.SPEAKER_BLUE_AUTO);
	}

	public Command autoScoreScoreSpeakerFarSideFirstShot() {
		return new ScoreSpeakerAutoCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				drivetrainSubsystem::getCurrentFieldChassisSpeeds,
				() -> drivetrainSubsystem.getState().Pose,
				ShootingConstants.SPEAKER_RED_AUTO_FIRST_SHOT,
				ShootingConstants.SPEAKER_BLUE_AUTO_FIRST_SHOT);
	}

	/**
	 * Command to bloop a note out of the shooter in the field +Y direction
	 *
	 * @return new command
	 */
	public Command bloopFirstNorth() {
		return new BloopCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				() -> drivetrainSubsystem.getState().Pose.getRotation(),
				Rotation2d.fromDegrees(90),
				Degrees.of(1),
				RotationsPerSecond.of(12));
	}

	/**
	 * Command to bloop a note out of the shooter in the field -Y direction
	 *
	 * @return new command
	 */
	public Command bloopFirstSouth() {
		return new BloopCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				() -> drivetrainSubsystem.getState().Pose.getRotation(),
				Rotation2d.fromDegrees(-90),
				Degrees.of(1),
				RotationsPerSecond.of(12));
	}

	/**
	 * Command to bloop a note out of the shooter toward the opposing alliance wall
	 *
	 * @return new command
	 */
	public Command bloopFirstEast() {
		return new BloopCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				() -> drivetrainSubsystem.getState().Pose.getRotation(),
				Rotation2d.fromDegrees(0),
				Degrees.of(1),
				RotationsPerSecond.of(12));
	}

	/**
	 * Command to bloop a note out of the shooter toward your alliance wall
	 *
	 * @return new command
	 */
	public Command bloopFirstWest() {
		return new BloopCommand(
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				() -> drivetrainSubsystem.getState().Pose.getRotation(),
				Rotation2d.fromDegrees(180),
				Degrees.of(1),
				RotationsPerSecond.of(12));
	}

	/**
	 * Command to intake a note into the turret
	 *
	 * @return new command
	 */
	public Command intakeToTurret() {
		return new IntakeCommand(intakeSubsystem, turretSubsystem)
				.deadlineFor(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kBlue, Seconds.one()))
				.andThen(
						ledSubsystem.runOnce(
								() -> ledSubsystem.setUpdater(
										(leds) -> leds.setAll(
												turretSubsystem.hasNote()
														? LEDConstants.NOTE_COLOR
														: Color.kBlack))));
	}

	/**
	 * Command to score in amp
	 *
	 * @return new command
	 */
	public Command scoreAmp() {
		return new ScoreAmpCommand(turretSubsystem, shooterSubsystem)
				.deadlineFor(new LEDMarqueeCommand(ledSubsystem, 70, 255, 0, 15, 0.08));
	}

	/**
	 * Command to put the turret in intake position
	 *
	 * @return new command
	 */
	public Command babyBird() {
		return new BabyBirdCommand(turretSubsystem, shooterSubsystem)
				.deadlineFor(new LEDBlinkCommand(ledSubsystem, BABYBIRD_COLOR, 0.1));
	}

	/**
	 * Comand to shoot a note into the middle of the field
	 *
	 * @param xSupplier X translation supplier
	 * @param ySupplier Y translation supplier
	 * @return new command
	 */
	public Command shootMid(Supplier<LinearVelocity> xSupplier, Supplier<LinearVelocity> ySupplier) {
		return new ShootTeleopCommand(
				drivetrainSubsystem,
				shooterSubsystem,
				turretSubsystem,
				ledSubsystem,
				xSupplier,
				ySupplier,
				STOCKPILE_MID_RED,
				STOCKPILE_MID_BLUE,
				STOCKPILE_INTERPOLATOR,
				0.3)
				.deadlineFor(
						(new LEDAlternateCommand(ledSubsystem, Color.kBlue, Color.kOrange, Seconds.of(0.1))));
	}
}
