package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimAutoCommand;
import frc.robot.commands.AutoAimTeleopCommand;
import frc.robot.commands.AutoEndIntakeCommand;
import frc.robot.commands.AutoStartIntakeCommand;
import frc.robot.subsystems.AmpShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutonomousBuilder {
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutonomousBuilder(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem,
      IntakeSubsystem intakeSubsystem, AmpShooterSubsystem ampShooterSubsystem,
      CommandSwerveDrivetrain commandSwerveDrivetrain, ElevatorSubsystem elevatorSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
  }

  public Command autoAimShoot_auto() {
    return new AutoAimAutoCommand(turretSubsystem, shooterSubsystem, commandSwerveDrivetrain);
  }

  public Command autoAimShoot_teleop() {
    return new AutoAimTeleopCommand(turretSubsystem, shooterSubsystem, commandSwerveDrivetrain);
  }

  public Command autoStartIntake() {
    return new AutoStartIntakeCommand(intakeSubsystem);
  }

  public Command autoEndIntake() {
    return new AutoEndIntakeCommand(intakeSubsystem);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
