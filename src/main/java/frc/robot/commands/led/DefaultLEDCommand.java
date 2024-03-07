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

  private static final int MODE_DS_DISCONNECT = 0;
  private static final int MODE_NOTE_IN_TURRET = 1;
  private static final int MODE_NOTE_IN_AMPER = 2;
  private static final int MODE_ROBOT_DISABLED = 3;
  private static final int MODE_DEFAULT = 4;

  private final LEDSubsystem ledSubsystem;
  private final BooleanSupplier noteInTurret;
  private final BooleanSupplier noteInAmper;

  public DefaultLEDCommand(LEDSubsystem ledSubsystem, BooleanSupplier noteInTurret, BooleanSupplier noteInAmper) {
    this.ledSubsystem = ledSubsystem;
    this.noteInTurret = noteInTurret;
    this.noteInAmper = noteInAmper;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    ledSubsystem.setUpdater(this::updateLeds);
  }

  private void updateLeds(LEDStrips strips) {
    switch(getMode()) {
      case MODE_DS_DISCONNECT:
        new LEDAlternateCommand(ledSubsystem, Color.kDarkRed, Color.kIndianRed, Seconds.of(0.5))
            .until(() -> getMode() != MODE_DS_DISCONNECT).schedule();
        break;
      case MODE_NOTE_IN_TURRET:
        strips.setAll(NOTE_COLOR);
        break;
      case MODE_NOTE_IN_AMPER:
        new LEDMarqueeCommand(ledSubsystem, 3, 255, 0, 15, .07)
            .until(() -> getMode() != MODE_NOTE_IN_AMPER).schedule();
        break;
      case MODE_ROBOT_DISABLED:
        new LEDAlternateCommand(ledSubsystem, Color.kBlue, Color.kOrange, Seconds.one())
            .until(() -> getMode() != MODE_ROBOT_DISABLED).schedule();
        break;
      default:
        // default state is off
        strips.setAll(Color.kBlack);
    }
  }

  /**
   * Gets the LED mode. This is used instead of putting the if block in {@link #updateLeds(LEDStrips)} so the mode can
   * be checked as a condition to cancel sub-commands
   * @return LED mode
   */
  private int getMode() {
    if (noteInTurret.getAsBoolean()) {
      return MODE_NOTE_IN_TURRET;
    } else if (noteInAmper.getAsBoolean()) {
      return MODE_NOTE_IN_AMPER;
    } else if (!DriverStation.isDSAttached()) {
      return MODE_DS_DISCONNECT;
    } else if (RobotState.isDisabled()) {
      return MODE_ROBOT_DISABLED;
    } else {
      return MODE_DEFAULT;
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
