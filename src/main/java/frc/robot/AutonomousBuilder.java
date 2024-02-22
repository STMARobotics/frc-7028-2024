package frc.robot;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousBuilder {
  private IntakeSubsystem intakeSubsystem;

  private ElevatorSubsystem elevatorSubsystem;
  private ShooterSubsystem shooterSubsystem;

  public AutonomousBuilder(IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  /*
   * public Command aimAndShoot() {
   * return new >>>INSERT COMMAND HERE<<<
   * }
   * 
   * 
   * public Command startIntake() {
   * return new >>>INSERT COMMAND HERE<<<
   * }
   * 
   * 
   * 
   * public Command shootDonut() {
   * return new >>>INSERT COMMAND HERE<<<
   * }
   */

}