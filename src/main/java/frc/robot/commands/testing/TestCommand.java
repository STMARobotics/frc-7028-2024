package frc.robot.commands.testing;

import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;

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
        if (timer.hasElapsed(2)) {
          intakeSubsystem.stop();
          if (timer.hasElapsed(5)) {
            teststate++;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case 1:
        intakeSubsystem.reverse();
        timer.start();
        if (timer.hasElapsed(2)) {
          intakeSubsystem.stop();
          if (timer.hasElapsed(5)) {
            teststate++;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case 2:
        shooterSubsystem.prepareToShoot(shootingSettings.getVelocity());
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
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
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          if (timer.hasElapsed(5)) {
            teststate++;
            timer.stop();
            timer.reset();
          }
        }
        break;
      case 5:
        turretSubsystem.eject();
        timer.start();
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          if (timer.hasElapsed(5)) {
            teststate++;
            timer.stop();
            timer.reset();
          }
        }
        break;
    }
  }

  public int getTestState() {
    return teststate;
  }

  @Override
  public boolean isFinished() {
    return teststate >= 5;
  }
}
