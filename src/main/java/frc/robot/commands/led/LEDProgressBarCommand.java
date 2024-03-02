package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TestMode;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {
  
  private final LEDSubsystem ledSubsystem;
  private final TestMode testMode;
  private boolean done;
  private int index;

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, TestMode testMode) {
    this.ledSubsystem = ledSubsystem;
    this.testMode = testMode;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {

    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
      for(int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        for(int index = 0; index < testMode.progressCounter; index++) {
            ledStrips.setLED(strip, index, Color.kBlue);
        }
      }
      ledStrips.refresh();
      done = index >= ledStrips.getStripSize();
    }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
