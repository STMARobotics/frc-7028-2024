package frc.robot.commands.testing;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {

  private final IntSupplier intSupplier;
  private final LEDSubsystem ledSubsystem;
  private int index;
  private double tests;

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, IntSupplier intSupplier) {
    this.ledSubsystem = ledSubsystem;
    this.intSupplier = intSupplier;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {

    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
    if (tests <= 7) {
      for (int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        for (index = 0; index < ledStrips.getStripSize(strip); index++) {
          if (index < tests * 2) {
            if (index < 1) {
              ledStrips.setLED(strip, index, Color.kRed);
            } else if (index < 2) {
              ledStrips.setLED(strip, index, Color.kOrange);
            } else if (index < 3) {
              ledStrips.setLED(strip, index, Color.kYellow);
            } else if (index < 4) {
              ledStrips.setLED(strip, index, Color.kLimeGreen);
            } else if (index < 5) {
              ledStrips.setLED(strip, index, Color.kBlue);
            } else if (index < 6) {
              ledStrips.setLED(strip, index, Color.kPurple);
            } else {
              ledStrips.setLED(strip, index, Color.kPink);
            }
          } else {
            ledStrips.setLED(strip, index, Color.kBlack);
          }
        }
      }
      ledStrips.refresh();
    } else {
      ledStrips.setAll(Color.kLimeGreen);
    }
  }

  @Override
  public void execute() {
    tests = intSupplier.getAsInt();
  }

  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setUpdater(null);
  }
}
