package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimShootCommand;
import frc.robot.commands.AutoStartIntakeCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutonomousBuilder {
  private IntakeSubsystem intakeSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private CommandSwerveDrivetrain driveTrain;

  public AutonomousBuilder(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      CommandSwerveDrivetrain driveTrain) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.driveTrain = driveTrain;
    
  NamedCommands.registerCommand("aimAndShoot", aimAndShoot());
  NamedCommands.registerCommand("shootDonut", shootDonut());
  NamedCommands.registerCommand("startIntake", startIntake());
  
  }

  public Command aimAndShoot() {
    return new AutoAimShootCommand(driveTrain, shooterSubsystem, turretSubsystem);
  }

  public Command startIntake() {
    return new AutoStartIntakeCommand(intakeSubsystem);
  }

  public Command shootDonut() {
    return new ManualShootCommand(turretSubsystem, shooterSubsystem);
  }
  
}