package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeToTurretCommand;
import frc.robot.commands.ScoreSpeakerCommand;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.telemetry.ShootingState;

/**
 * Non-static command factory for creating commands. See 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories
 */
public class AutoCommands {
  
  private final AmperSubsystem amperSubsystem;
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Consumer<ShootingState> shootingTelemetryConsumer;

  public AutoCommands(AmperSubsystem amperSubsystem, CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem,
      Consumer<ShootingState> shootingTelemetryConsumer) {
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shootingTelemetryConsumer = shootingTelemetryConsumer;
  }

  /**
   * Registers named commands for PathPlanner
   */
  public void registerPPNamedCommands() {
    NamedCommands.registerCommand("scoreSpeaker", scoreSpeaker());
    NamedCommands.registerCommand("intake", intakeToTurret());
  }

  /**
   * Command to automatically score in the speaker
   * @return new command
   */
  public Command scoreSpeaker() {
    return new ScoreSpeakerCommand(drivetrainSubsystem, shooterSubsystem, turretSubsystem, shootingTelemetryConsumer);
  }

  /**
   * Command to intake a note into the turret
   * @return new command
   */
  public Command intakeToTurret() {
    return new IntakeToTurretCommand(intakeSubsystem, turretSubsystem, amperSubsystem);
  }

}