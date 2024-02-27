package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAimShootCommand;
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

  public Pose2d poseSupplier() {
    return driveTrain.getState().Pose;
  }

  public Command aimAndShoot() {
    return new AutoAimShootCommand(shooterSubsystem, turretSubsystem, poseSupplier());
  }

  public Command startIntake() {
    return Commands.startEnd(() -> intakeSubsystem.intake(), null, intakeSubsystem);
  }

  public Command shootDonut() {
    return new ManualShootCommand(turretSubsystem, shooterSubsystem);
  }
  
}