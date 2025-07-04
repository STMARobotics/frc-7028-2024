package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    teststate = 0;
    hasStopped = false;
  }

  @Override
  public void execute() {
    switch (teststate) {
      case 0:
        if (!hasStopped) {
          intakeSubsystem.runRollers(RotationsPerSecond.of(20));
          timer.start();
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          intakeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4) && hasStopped) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }

        break;

      case 1:
        if (!hasStopped) {
          intakeSubsystem.runRollers(RotationsPerSecond.of(-20));
          timer.start();
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          intakeSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(4)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }

        break;

      case 2:
        if (!hasStopped) {
          shooterSubsystem.prepareToShoot(RotationsPerSecond.of(40));
        }
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          timer.start();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 3:
        if (!hasStopped) {
          shooterSubsystem.prepareToShoot(RotationsPerSecond.of(-40));
        }
        if (shooterSubsystem.isReadyToShoot()) {

          shooterSubsystem.stop();
          timer.start();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 4:
        if (!hasStopped) {
          turretSubsystem.shoot();
          timer.start();
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          turretSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 5:
        if (!hasStopped) {
          turretSubsystem.eject();
          timer.start();
        }
        if (timer.hasElapsed(2) && !hasStopped) {
          turretSubsystem.stop();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 6:
        if (!hasStopped) {
          turretSubsystem.moveToYawPosition(Degrees.of(-160));
        }
        if (turretSubsystem.isAtYawTarget()) {
          turretSubsystem.prepareToIntake();
          timer.start();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;

      case 7:
        if (!hasStopped) {
          turretSubsystem.moveToPitchPosition(Degrees.of(10));
        }
        if (turretSubsystem.isAtPitchTarget()) {
          turretSubsystem.prepareToIntake();
          timer.start();
          hasStopped = true;
        }
        if (timer.hasElapsed(3)) {
          teststate++;
          timer.stop();
          timer.reset();
          hasStopped = false;
        }
        break;
      default:
        // NO-OP
    }
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
  }
}
