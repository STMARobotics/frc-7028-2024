package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreSpeakerAutoCommand;
import frc.robot.commands.ScoreSpeakerOffsetAutoCommand;
import frc.robot.commands.led.LEDAlternateCommand;
import frc.robot.commands.led.LEDMarqueeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;


/**
 * Non-static command factory for creating commands. See 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories
 */
public class AutoCommands {
  
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;

  public AutoCommands(CommandSwerveDrivetrain drivetrainSubsystem, ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  /**
   * Registers named commands for PathPlanner
   */
  public void registerPPNamedCommands() {
    NamedCommands.registerCommand("intake", intakeToTurret());
    NamedCommands.registerCommand("scoreSpeakerWhileMoving", autoScoreSpeaker());
    NamedCommands.registerCommand("firstAutoShot", autoScoreSpeakerOffset());
  }

  /**
   * Command to automatically score in the speaker while moving
   * @return new command
   */
  public Command autoScoreSpeaker() {
    return new ScoreSpeakerAutoCommand(shooterSubsystem, turretSubsystem, ledSubsystem,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds, () -> drivetrainSubsystem.getState().Pose);
  }

  public Command autoScoreSpeakerOffset() {
    return new ScoreSpeakerOffsetAutoCommand(shooterSubsystem, turretSubsystem, ledSubsystem, 
    drivetrainSubsystem::getCurrentFieldChassisSpeeds, () -> drivetrainSubsystem.getState().Pose);
  }

  /**
   * Command to intake a note into the turret
   * @return new command
   */
  public Command intakeToTurret() {
    return new IntakeCommand(intakeSubsystem, turretSubsystem)
        .deadlineWith(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kBlue, Seconds.one()))
        .andThen(ledSubsystem.runOnce(() -> ledSubsystem.setUpdater(
              (leds) -> leds.setAll(turretSubsystem.hasNote() ? LEDConstants.NOTE_COLOR : Color.kBlack))));
  }

  /**
   * Command to score in amp
   * @return new command
   */
  public Command scoreAmp() {
    return new ScoreAmpCommand(turretSubsystem, shooterSubsystem)
        .deadlineWith(new LEDMarqueeCommand(ledSubsystem, 70, 255, 0, 15, 0.08));
  }

}
