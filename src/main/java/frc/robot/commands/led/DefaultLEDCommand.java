package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Default LED command to make the LEDs dance or indicate when a note is being held.
 */
public class DefaultLEDCommand extends Command {

  private enum LEDMode {
    MODE_DS_DISCONNECT,
    MODE_NOTE_IN_TURRET,
    MODE_ROBOT_DISABLED,
    MODE_DEFAULT,
    MODE_TEST;
  }

  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier noteInTurret;

  public DefaultLEDCommand(LEDSubsystem ledSubsystem, BooleanSupplier noteInTurret) {
    this.ledSubsystem = ledSubsystem;
    this.noteInTurret = noteInTurret;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    ledSubsystem.setUpdater(this::updateLeds);
  }

  private void updateLeds(LEDStrips strips) {
    // DO NOT schedule commands here, this is on a the LED callback thread and the scheduler is not threadsafe.
    // execute() handles all of the cases where the LEDs should be on. If this is called, the LEDs should be off.
    strips.setAll(Color.kBlack);
  }

  @Override
  public void execute() {
    // Based on the mode, schedule a command to do the animation.
    switch(getMode()) {
      case MODE_DS_DISCONNECT:
        new LEDAlternateCommand(ledSubsystem, Color.kDarkRed, Color.kIndianRed, Seconds.of(0.5))
            .until(() -> getMode() != LEDMode.MODE_DS_DISCONNECT).schedule();
        break;
      case MODE_NOTE_IN_TURRET:
        new LEDMarqueeCommand(ledSubsystem, 3, 255, 0, 15, .07)
            .until(() -> getMode() != LEDMode.MODE_NOTE_IN_TURRET).schedule();
        break;
      case MODE_ROBOT_DISABLED:
        new LEDAlternateCommand(ledSubsystem, Color.kBlue, Color.kOrange, Seconds.one())
            .until(() -> getMode() != LEDMode.MODE_ROBOT_DISABLED).schedule();
        break;
      case MODE_TEST:
        new LEDAlternateCommand(ledSubsystem, Color.kBlack, NOTE_COLOR, Seconds.one())
          .until(() -> getMode() != LEDMode.MODE_TEST).schedule();
        break;
      default:
        // Don't do anything for default. LEDs will go off.
    }
  }

  /**
   * Gets the LED mode. This is used instead of putting the if block in {@link #updateLeds(LEDStrips)} so the mode can
   * be checked as a condition to cancel sub-commands
   * @return LED mode
   */
  private LEDMode getMode() {
    if (noteInTurret.getAsBoolean()) {
      return LEDMode.MODE_NOTE_IN_TURRET;
    } else if (!DriverStation.isDSAttached()) {
      return LEDMode.MODE_DS_DISCONNECT;
    } else if (RobotState.isDisabled() && !RobotState.isTest()) {
      return LEDMode.MODE_ROBOT_DISABLED;
    } else if (RobotState.isTest()) {
      return LEDMode.MODE_TEST;
    } else {
      return LEDMode.MODE_DEFAULT;
    }
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setUpdater(null);
  }
  
}
