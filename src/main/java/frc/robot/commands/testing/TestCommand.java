package frc.robot.commands.testing;

import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TestCommand extends Command {
  
  private final LEDSubsystem ledSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final AmperSubsystem amperSubsystem;

  public int teststate = 0;

  private Timer timer = new Timer();

  ShuffleboardTab tab = Shuffleboard.getTab("Testing");

  public TestCommand(
    LEDSubsystem ledSubsystem, 
    IntakeSubsystem intakeSubsystem, 
    ShooterSubsystem shooterSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    TurretSubsystem turretSubsystem,
    AmperSubsystem amperSubsystem) {
      this.ledSubsystem = ledSubsystem;
      this.intakeSubsystem = intakeSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.elevatorSubsystem = elevatorSubsystem;
      this.turretSubsystem = turretSubsystem;
      this.amperSubsystem = amperSubsystem;

    addRequirements(ledSubsystem, intakeSubsystem, shooterSubsystem, elevatorSubsystem, turretSubsystem, amperSubsystem);
  }

  @Override
  public void initialize() {

    tab.addNumber("Number of Tests Run", () -> getTestState());
    tab.add("Start LED Progress Bar", new LEDProgressBarCommand(ledSubsystem, this)).withPosition(0, 0);
  }
  
  @Override
  public void execute() {
    var shootingSettings = SHOOTER_INTERPOLATOR.calculate(1.34);
    switch (teststate) {
      case 0:
        timer.reset();
        intakeSubsystem.intake();
        timer.start();
        if (timer.hasElapsed(2)) {
          intakeSubsystem.stop();
          teststate++;
          break;
        }
      case 1:
        timer.reset();
        intakeSubsystem.reverse();
        timer.start();
        if (timer.hasElapsed(2)) {
          intakeSubsystem.stop();
          teststate++;
          break;
        }
      case 2:
        shooterSubsystem.prepareToShoot(shootingSettings.getVelocity());
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          teststate++;
          break;
        }
      case 3:
        shooterSubsystem.reverse();
        if (shooterSubsystem.isReadyToShoot()) {
          shooterSubsystem.stop();
          teststate++;
          break;
        }
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
          break;
        }
      case 5:
        timer.reset();
        amperSubsystem.intake();
        timer.start();
        if (timer.hasElapsed(2)) {
          amperSubsystem.stop();
          teststate++;
          break;
        }
      case 6:
        timer.reset();  
        turretSubsystem.shoot();
        timer.start();
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          teststate++;
          break;
        }
      case 7:
        timer.reset();  
        turretSubsystem.eject();
        timer.start();
        if (timer.hasElapsed(2)) {
          turretSubsystem.stop();
          teststate++;
          break;
        } 
      }
  }

  public double getTestState() {
    return teststate;
  }

  @Override
  public boolean isFinished() {
    return teststate == 7;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}

