package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.IntakeToAmperCommand;
import frc.robot.commands.IntakeToTurretCommand;
import frc.robot.commands.LoadAmperCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.ScoreSpeakerAutoCommand;
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

  public AutoCommands(AmperSubsystem amperSubsystem, CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem,
      LEDSubsystem ledSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.amperSubsystem = amperSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
  }

  /**
   * Registers named commands for PathPlanner
   */
  public void registerPPNamedCommands() {
    NamedCommands.registerCommand("intake", intakeToTurret());
    NamedCommands.registerCommand("scoreSpeakerWhileMoving", autoScoreSpeaker());
    NamedCommands.registerCommand("scoreSpeakerWhileMoving-Long", autoScoreSpeakerLong());
  }

  /**
   * Command to stop moving and automatically score in the speaker
   * @return new command
   * @deprecated not quite ready to remove, but replaced with {@link #autoScoreSpeaker}
   */
  public Command scoreSpeaker() {
    return new ScoreSpeakerCommand(
        drivetrainSubsystem, shooterSubsystem, turretSubsystem, ledSubsystem, elevatorSubsystem::isParked);
  }

  /**
   * Command to automatically score in the speaker while moving
   * @return new command
   */
  public Command autoScoreSpeaker() {
    return new ScoreSpeakerAutoCommand(shooterSubsystem, turretSubsystem, ledSubsystem,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds, () -> drivetrainSubsystem.getState().Pose,
        elevatorSubsystem::isParked, Degrees.zero());
  }

  /**
   * Command to automatically score in the speaker while moving, with a boost to the angle for longer shots
   * @return new command
   */
  private Command autoScoreSpeakerLong() {
    return new ScoreSpeakerAutoCommand(shooterSubsystem, turretSubsystem, ledSubsystem,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds, () -> drivetrainSubsystem.getState().Pose,
        elevatorSubsystem::isParked, Degrees.of(3.0));
  }

  /**
   * Command to intake a note into the turret
   * @return new command
   */
  public Command intakeToTurret() {
    return new IntakeToTurretCommand(intakeSubsystem, turretSubsystem, amperSubsystem)
        .deadlineWith(new LEDAlternateCommand(ledSubsystem, NOTE_COLOR, Color.kBlue, Seconds.one()))
        .andThen(ledSubsystem.runOnce(() -> ledSubsystem.setUpdater(
              (leds) -> leds.setAll(turretSubsystem.hasNote() ? LEDConstants.NOTE_COLOR : Color.kBlack))));
  }

  /**
   * Command to intake a note to the amper
   * @return new command
   */
  public Command intakeToAmper() {
    return new IntakeToAmperCommand(intakeSubsystem, amperSubsystem)
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
