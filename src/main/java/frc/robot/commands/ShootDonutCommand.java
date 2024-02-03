package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Basic command to position elevator and wrist, and then shoot
 */
public class ShootDonutCommand extends Command {

  private static final double SHOOT_TIME = 0.25;

  private final ShooterSubsystem shooterSubsystem;
  private final Timer shootTimer = new Timer();


  protected double elevatorMeters;
  protected double wristRadians;
  protected double shooterRPS;
  
  private boolean isShooting = false;


  /**
   * Constructor
   * @param elevatorMeters position of elevator in meters
   * @param wristRadians position of wrist in radians
   * @param shooterRPS velocity of shooter in rotations per second
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param shooterSubsystem shooter
   */
  public ShootDonutCommand(double elevatorMeters, double wristRadians, double shooterRPS, Color ledColor,
      ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.shooterRPS = shooterRPS;

    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shootTimer.reset();
    isShooting = false;
    shooterSubsystem.activeStop();
  }

  @Override
  public void execute() {
    if (isShooting) {
      shooterSubsystem.spinShooterWheel(shooterRPS);
      shootTimer.start();
      isShooting = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isShooting && shootTimer.hasElapsed(SHOOT_TIME);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.actuatorStop();
    if (isShooting) {
      shooterSubsystem.stop();
    } else {
      shooterSubsystem.activeStop();
    }
    shootTimer.stop();
  }

}