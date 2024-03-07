package frc.robot.commands.testing;

import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;

  private int teststate = 0;

  private Timer timer = new Timer();



  public TestCommand(
    IntakeSubsystem intakeSubsystem, 
    ShooterSubsystem shooterSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    TurretSubsystem turretSubsystem,
    AmperSubsystem amperSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.elevatorSubsystem = elevatorSubsystem;
      this.turretSubsystem = turretSubsystem;
      this.amperSubsystem = amperSubsystem;

    addRequirements(intakeSubsystem, shooterSubsystem, elevatorSubsystem, turretSubsystem, amperSubsystem);
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
          teststate++;
          timer.reset();
        }
        break;
      case 1:
        intakeSubsystem.reverse();
        timer.start();
        if (timer.hasElapsed(2)) {
          intakeSubsystem.stop();
          teststate++;
          timer.reset();
        }
        break;
      case 2:
        shooterSubsystem.prepareToShoot(shootingSettings.getVelocity());
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          teststate++;
        }
        break;
      case 3:
        shooterSubsystem.reverse();
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          teststate++;
        }
        break;
      case 4:
        turretSubsystem.prepareToTrap(elevatorSubsystem::isParked);
        if (turretSubsystem.isInTrapPosition()) {
          elevatorSubsystem.prepareToTrap(turretSubsystem::isClearOfElevator);
        }
        if (elevatorSubsystem.isAtTarget()) {
          elevatorSubsystem.park(turretSubsystem::isClearOfElevator);
        }
        if (elevatorSubsystem.isParked()) {
          turretSubsystem.stop();
          teststate++;
        }
        break;
      case 5:
        amperSubsystem.intake();
        timer.start();
        if (timer.hasElapsed(2)) {
          amperSubsystem.stop();
          teststate++;
          timer.reset();
        }
        break;
      case 6:
        turretSubsystem.shoot();
        timer.start();
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          teststate++;
          timer.reset();
        }
        break;
      case 7: 
        turretSubsystem.eject();
        timer.start();
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          teststate++;
          timer.reset();
        } 
        break;
      }
  }

  public int getTestState() {
    return teststate;
  }

  @Override
  public boolean isFinished() {
    return teststate == 7;
  }
}

