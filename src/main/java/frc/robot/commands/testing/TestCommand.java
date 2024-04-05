package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;

  private boolean hasStopped = false;
  private int teststate = 0;

  private Timer timer = new Timer();

  public TestCommand(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(intakeSubsystem, shooterSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(1.34);
    switch (teststate) {

      case 0:
        intakeSubsystem.intake();
        timer.start();
        if (timer.hasElapsed(2) && !hasStopped) {
          intakeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }

        break;

      case 1:
        intakeSubsystem.reverse();
        timer.start();
        if (timer.hasElapsed(2) && !hasStopped) {
          intakeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }

        break;

      case 2:
        shooterSubsystem.prepareToShoot(shootingSettings.getVelocity());
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          timer.start();
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
        }
        break;

      case 3:
        shooterSubsystem.reverse();
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          timer.start();
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
        }
        break;

      case 4:
        turretSubsystem.shoot();
        timer.start();
        if (timer.hasElapsed(2) && !hasStopped) {
          turretSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 5:
        turretSubsystem.eject();
        timer.start();
        if (timer.hasElapsed(2) && !hasStopped) {
          turretSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(5)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 6:
        turretSubsystem.moveToYawPosition(Degrees.of(135));
        if (turretSubsystem.isAtYawTarget()) {
          turretSubsystem.prepareToIntake();
          timer.start();
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
        }

      case 7:
        turretSubsystem.moveToYawPosition(Degrees.of(225));
        if (turretSubsystem.isAtYawTarget()) {
          turretSubsystem.prepareToIntake();
          timer.start();
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
        }

      case 8:
        turretSubsystem.moveToPitchPosition(Degrees.of(30));
        if (turretSubsystem.isAtPitchTarget()) {
          turretSubsystem.prepareToIntake();
          timer.start();
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
        }
    }
  }

  public void addTestState() {
    teststate++;
  }

  public int getTestState() {
    return teststate;
  }

  public boolean getHasStopped() {
    return hasStopped;
  }

  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    turretSubsystem.stop();
    intakeSubsystem.stop();
    turretSubsystem.prepareToIntake();
    teststate = 0;
  }
}
