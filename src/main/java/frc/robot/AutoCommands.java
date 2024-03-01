package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoIntakeToAmperCommand;
import frc.robot.commands.AutoIntakeToTurretCommand;
import frc.robot.commands.LoadAmperCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreSpeakerCommand;
import frc.robot.commands.led.LEDAlternateCommand;
import frc.robot.commands.led.LEDMarqueeCommand;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

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
  private final LEDSubsystem ledSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LimelightHelpers limelightHelpers;

  public AutoCommands(
    AmperSubsystem amperSubsystem, 
    CommandSwerveDrivetrain drivetrainSubsystem,
    ShooterSubsystem shooterSubsystem, 
    TurretSubsystem turretSubsystem, 
    IntakeSubsystem intakeSubsystem,
    LEDSubsystem ledSubsystem, 
    ElevatorSubsystem elevatorSubsystem,
    LimelightHelpers limelightHelpers) {
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.limelightHelpers = limelightHelpers;
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
    return new ScoreSpeakerCommand(drivetrainSubsystem, shooterSubsystem, turretSubsystem, ledSubsystem);
  }

  /**
   * Command to intake a note into the turret
   * @return new command
   */
  public Command intakeToTurret() {
    return new AutoIntakeToTurretCommand(intakeSubsystem, turretSubsystem, amperSubsystem, drivetrainSubsystem, limelightHelpers)
        .deadlineWith(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kBlue, Seconds.one()));
  }

  /**
   * Command to intake a note to the amper
   * @return new command
   */
  public Command intakeToAmper() {
    return new AutoIntakeToAmperCommand(intakeSubsystem, amperSubsystem, drivetrainSubsystem, limelightHelpers)
        .deadlineWith(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kGreen, Seconds.one()));
  }

  /**
   * Command to transfer note from turret to amper
   * @return new command
   */
  public Command transferToAmper() {
    return new LoadAmperCommand(amperSubsystem, turretSubsystem, intakeSubsystem, elevatorSubsystem)
        .deadlineWith(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kGreen, Seconds.of(0.5)));
  }

  /**
   * Command to score in amp
   * @return new command
   */
  public Command scoreAmp() {
    return new ScoreAmpCommand(elevatorSubsystem, amperSubsystem, turretSubsystem)
        .deadlineWith(new LEDMarqueeCommand(ledSubsystem, 70, 255, 0, 15, 0.08));
  }

}
