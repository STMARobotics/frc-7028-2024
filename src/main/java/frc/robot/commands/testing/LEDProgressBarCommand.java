package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.IntSupplier;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {

  private final IntSupplier intSupplier;
  private final LEDSubsystem ledSubsystem;
  private int tests;
  private boolean ledsOn = true;
  private int hasBlinked = 0;

  private Timer timer = new Timer();

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, IntSupplier intSupplier) {
    this.ledSubsystem = ledSubsystem;
    this.intSupplier = intSupplier;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    ledsOn = true;
    hasBlinked = 0;
    tests = 0;

    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) {

    if (tests <= 21) {
      for (int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        int index;
        for (index = 0; index < ledStrips.getStripSize(strip); index++) {
          if (index < tests) {
            if (index < 3) {
              ledStrips.setLED(strip, index, Color.kRed);
            } else if (index < 6) {
              ledStrips.setLED(strip, index, Color.kOrange);
            } else if (index < 9) {
              ledStrips.setLED(strip, index, Color.kYellow);
            } else if (index < 12) {
              ledStrips.setLED(strip, index, Color.kLimeGreen);
            } else if (index < 15) {
              ledStrips.setLED(strip, index, Color.kBlue);
            } else if (index < 18) {
              ledStrips.setLED(strip, index, Color.kPurple);
            } else if (index < 21) {
              ledStrips.setLED(strip, index, Color.kPink);
            } else if (index < 24) {
              ledStrips.setLED(strip, index, Color.kWhite);
            }
          } else {
            ledStrips.setLED(strip, index, Color.kBlack);
          }
        }
      }
      ledStrips.refresh();
    } else if (hasBlinked <= 6) {
      timer.start();
      if (timer.advanceIfElapsed(0.5)) {
        if (ledsOn) {
          ledsOn = false;
          ledStrips.setAll(Color.kBlack);
        } else {
          ledsOn = true;
          ledStrips.setAll(Color.kOrange);
        }
        hasBlinked++;
      }
    }
  }

  @Override
  public void execute() {
    tests = (intSupplier.getAsInt() * 3);
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
