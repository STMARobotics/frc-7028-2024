package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to handle LED lighting
 */
public class LEDSubsystem extends SubsystemBase {
  
  private static final int LED_COUNT = 128;
  private static final int STRIP_COUNT = 4;
  private static final int STRIP_SIZE = LED_COUNT / STRIP_COUNT;

  private final AtomicReference<Consumer<LEDStrips>> ledUpdateConsumer = new AtomicReference<Consumer<LEDStrips>>(null);
  private final Notifier ledNotifier;
  private final LEDStripMethods ledStripMethods = new LEDStripMethods();

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  public LEDSubsystem() {
    leds.setLength(LED_COUNT);
    leds.setData(buffer);
    leds.start();
    ledNotifier = new Notifier(() ->  {
      var value = ledUpdateConsumer.get();
      if (value == null) {
        // There is no updater, just turn the LEDs off
        ledStripMethods.setAll(Color.kBlack);
      } else {
        // Call the consumer to update the LEDs on this notifier thread
        value.accept(ledStripMethods);
      }
    });
    ledNotifier.setName("LEDs");
    ledNotifier.startPeriodic(0.02);
  }

  /**
   * Sets the current LED update callback. Pass an LEDStrips consumer that will be called on a background thread when
   * it's time to refresh. The updater should update the LEDs with a "set" method and then call {@link #refresh()},
   * unless the method is documented to perform a refresh automatically.
   * @param updater consumer that gets called when it's time to refresh
   */
  public void setUpdater(Consumer<LEDStrips> updater) {
    ledUpdateConsumer.set(updater);
  }

  /**
   * Calculates the index for an LED on a strip. The strips serpentine - index 0 and 2 start at the bottom of the robot,
   * 1 and 3 start at the top.
   * @param stripId ID of the strip [0,3]
   * @param ledId ID of the LED on the strip, always at the bottom of the robot
   * @return LED index in the buffer
   */
  private int calculateUpdateIndex(int stripId, int ledId) {
    int firstId = stripId * STRIP_SIZE;
    int index = stripId % 2 == 0 ? firstId + ledId : firstId + STRIP_SIZE - ledId - 1;
    return MathUtil.clamp(index, 0, LED_COUNT - 1);
  }

  /**
   * Implementation of {@link LEDStrips} for the LEDs managed by this subsystem
   */
  private class LEDStripMethods implements LEDStrips {
    
    public void setLED(int stripId, int ledId, Color color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    public void setLED(int stripId, int ledId, Color8Bit color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    public void setHSV(int stripId, int ledId, int h, int s, int v) {
      buffer.setHSV(calculateUpdateIndex(stripId, ledId), h, s, v);
    }

    public void setRGB(int stripId, int ledId, int r, int g, int b) {
      buffer.setRGB(calculateUpdateIndex(stripId, ledId), r, g, b);
    }
      
    public void refresh() {
      leds.setData(buffer);
    }

    public void setAll(Color color) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setLED(i, color);
      }
      refresh();
    }

    public void setAll(int r, int g, int b) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setRGB(i, r, g, b);
      }
      refresh();
    }

    public void setLEDSegments(Color color, boolean... segmentValues) {
      int ledsPerStatus = LEDSubsystem.STRIP_SIZE / segmentValues.length;
      for(int stripId = 0; stripId < LEDSubsystem.STRIP_COUNT; stripId++) {
        int ledIndex = 0;
        for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
          for(;ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
            setLED(stripId, ledIndex, segmentValues[segmentId] ? color : Color.kBlack);
          }
        }
      }
      refresh();
    }

    @Override
    public int getStripCount() {
      return STRIP_COUNT;
    }

    @Override
    public int getStripSize() {
      return STRIP_SIZE;
    }
  }

}
