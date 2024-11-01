package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * This command bloops a note out of the shooter at a fixed speed and turret pitch. It will turn the
 * turret to a field oriented angle, flipping it when on the RED alliance.
 */
public class BloopCommand extends Command {

  private final ShooterSubsystem shooter;
  private final TurretSubsystem turretSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Supplier<Rotation2d> rotationSupplier;

  // Reusable object to prevent reallocation (to reduce memory pressure)
  private final MutAngle turretYawTarget = Rotations.mutable(0);

  private final Rotation2d fieldOrientedTurretRotation;
  private final Angle turretPitch;
  private final AngularVelocity shooterVelocity;

  private Rotation2d allianceFieldOrientedTurretAngle;
  private boolean isShooting = false;

  /**
   * Constructor
   *
   * @param shooter shooter
   * @param turretSubsystem turret
   * @param ledSubsystem LED subsystem
   * @param poseSupplier supplier for the robot's current pose
   * @param fieldOrientedTurretAngle the field oriented angle to turn the turret. Specify the angle
   *          for the BLUE alliance, and it will be flipped when on RED (like PathPlanner)
   * @param turretPitch the pitch angle for the turret
   * @param shooterVelocity shooter wheel velocity
   */
  public BloopCommand(
      ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<Rotation2d> rotationSupplier,
      Rotation2d fieldOrientedTurretAngle,
      Angle turretPitch,
      AngularVelocity shooterVelocity) {
    this.shooter = shooter;
    this.turretSubsystem = turretSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedTurretRotation = fieldOrientedTurretAngle;
    this.turretPitch = turretPitch;
    this.shooterVelocity = shooterVelocity;

    addRequirements(shooter, turretSubsystem);
  }

  @Override
  public void initialize() {
    allianceFieldOrientedTurretAngle = fieldOrientedTurretRotation;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      // Flip the angle for the RED alliance
      allianceFieldOrientedTurretAngle = Rotation2d.fromDegrees(180).minus(allianceFieldOrientedTurretAngle);
    }

    isShooting = false;
  }

  @Override
  public void execute() {
    var robotRotation = rotationSupplier.get();

    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(allianceFieldOrientedTurretAngle.minus(robotRotation).getRotations(), Rotations);

    // Set the turret position and shooter speed
    turretSubsystem.moveToShootingYawPosition(turretYawTarget);
    turretSubsystem.moveToPitchPosition(turretPitch);
    shooter.prepareToShoot(shooterVelocity);

    // Calculate ready state
    var isShooterReady = shooter.isReadyToShoot();
    var isPitchReady = turretSubsystem.isAtPitchTarget();
    var isYawReady = turretSubsystem.isAtYawTarget();
    if (isShooterReady && isPitchReady && isYawReady && TurretSubsystem.isYawInShootingRange(turretYawTarget)) {
      // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot
      // and start timer
      turretSubsystem.shoot();
      isShooting = true;
    }

    // Update LEDs with ready state
    if (isShooting) {
      ledSubsystem.setUpdater(l -> l.setAll(kGreen));
    } else {
      ledSubsystem.setUpdater(l -> l.setLEDSegments(kBlue, isShooterReady, isPitchReady, isYawReady));
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
