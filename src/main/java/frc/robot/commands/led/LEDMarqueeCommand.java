package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Runs an LED marquee effect */
public class LEDMarqueeCommand extends Command {
  
  private static final int MARQUEE_SIZE = 16;
  private final LEDSubsystem ledSubsystem;
  private final int hue;
  private final int saturation;
  private final int minValue;
  private final int valueStep;
  private final double duration;

  private final Timer timer = new Timer();
  
  private int frame = 0;

  /**
   * Constructor
   * @param ledSubsystem LED subsystem
   * @param hue hue
   * @param saturation saturation
   * @param minValue lowest value, will be increased by valueStep
   * @param valueStep how much to increase the color value on each step
   * @param duration duration of each frame
   */
  public LEDMarqueeCommand(
      LEDSubsystem ledSubsystem, int hue, int saturation, int minValue, int valueStep, double duration) {
    this.ledSubsystem = ledSubsystem;
    this.hue = hue;
    this.saturation = saturation;
    this.minValue = minValue;
    this.valueStep = valueStep;
    this.duration = duration;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
    if (timer.advanceIfElapsed(duration)) {
      if (frame == 0) {
        frame = ledStrips.getStripSize();
      }
      for(int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        for(int index = 0; index < ledStrips.getStripSize(); index++) {
          int value = minValue + ((index + frame) % MARQUEE_SIZE) * valueStep;
          ledStrips.setHSV(strip, index, hue, saturation, value);
        }
      }
      frame--;
    }
    ledStrips.refresh();
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
