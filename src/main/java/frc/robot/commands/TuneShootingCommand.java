package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE_TELE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED_TELE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Testing command for tuning shots. This is not intended to be used in-game. This command reads
 * from the NetworkTables to get shooter pitch, velocity, and yaw.
 */
public class TuneShootingCommand extends Command {

	private final TurretSubsystem turretSubsystem;
	private final ShooterSubsystem shooterSubsystem;
	private final LEDSubsystem ledSubsystem;
	private final Supplier<Pose2d> poseSupplier;

	private final DoubleEntry pitchSubscriber;
	private final DoubleEntry topVelocitySubscriber;
	private final DoubleEntry yawSubscriber;
	private final DoubleEntry bottomVelocitySubscriber;
	private final DoublePublisher distancePublisher;

	private boolean shooting = false;
	private Translation2d speakerTranslation;

	private MutAngle pitchMeasure = Radian.mutable(0);
	private MutAngle yawMeasure = Radian.mutable(0);
	private MutAngularVelocity topVelocityMeasure = RotationsPerSecond.mutable(0);
	private MutAngularVelocity bottomVelocityMeasure = RotationsPerSecond.mutable(0);

	public TuneShootingCommand(
			TurretSubsystem turretSubsystem,
			ShooterSubsystem shooterSubsystem,
			LEDSubsystem ledSubsystem,
			Supplier<Pose2d> poseSupplier) {

		this.turretSubsystem = turretSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.ledSubsystem = ledSubsystem;
		this.poseSupplier = poseSupplier;

		var nt = NetworkTableInstance.getDefault();
		var table = nt.getTable("Tune Shoot");
		distancePublisher = table.getDoubleTopic("Speaker Distance").publish();
		pitchSubscriber = table.getDoubleTopic("Pitch (degrees)").getEntry(0.0);
		pitchSubscriber.set(0.0);
		topVelocitySubscriber = table.getDoubleTopic("Velocity Top (RPS)").getEntry(0.0);
		topVelocitySubscriber.set(0.0);
		bottomVelocitySubscriber = table.getDoubleTopic("Velocity Bottom (RPS)").getEntry(0.0);
		bottomVelocitySubscriber.set(0);
		yawSubscriber = table.getDoubleTopic("Yaw (Degrees)").getEntry(180.0);
		yawSubscriber.set(180.0);

		addRequirements(turretSubsystem, shooterSubsystem, ledSubsystem);
	}

	@Override
	public void initialize() {
		shooting = false;
		var alliance = DriverStation.getAlliance();
		speakerTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? SPEAKER_BLUE_TELE : SPEAKER_RED_TELE;
	}

	@Override
	public void execute() {
		var turretDistanceToSpeaker = TurretSubsystem.getTurretTranslation(poseSupplier.get())
				.getDistance(speakerTranslation);
		distancePublisher.accept(turretDistanceToSpeaker);

		turretSubsystem.moveToPitchPosition(
				pitchMeasure.mut_replace(pitchSubscriber.get(0.0), Degrees));
		turretSubsystem.moveToShootingYawPosition(
				yawMeasure.mut_replace(yawSubscriber.get(180.0), Degrees));
		shooterSubsystem.spinShooterWheels(
				topVelocityMeasure.mut_replace(topVelocitySubscriber.get(0.0), RotationsPerSecond),
				bottomVelocityMeasure.mut_replace(bottomVelocitySubscriber.get(10), RotationsPerSecond));
		var turretReady = turretSubsystem.isAtYawAndPitchTarget();
		var shooterReady = shooterSubsystem.isReadyToShoot();
		ledSubsystem.setUpdater(l -> l.setLEDSegments(NOTE_COLOR, turretReady, shooterReady));
		if (shooting || (turretReady && shooterReady)) {
			turretSubsystem.shoot();
			shooting = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		turretSubsystem.stop();
		shooterSubsystem.stop();
	}
}
