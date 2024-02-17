package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimShootCommand;
import frc.robot.subsystems.AmpShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoBuilder {
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain;

  public AutoBuilder(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem,
      IntakeSubsystem intakeSubsystem, AmpShooterSubsystem ampShooterSubsystem,
      CommandSwerveDrivetrain commandSwerveDrivetrain, ElevatorSubsystem elevatorSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
  }

  public Command autoAimShoot() {
    return new AutoAimShootCommand(commandSwerveDrivetrain, turretSubsystem, shooterSubsystem);
  }
  
}
