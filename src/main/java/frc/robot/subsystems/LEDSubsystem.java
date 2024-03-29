package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static frc.robot.Constants.LEDConstants.DEVICE_ID;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

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
  
  private static final int[] STRIP_SIZES = {20, 18, 20, 33};
  private static final int LED_COUNT = Arrays.stream(STRIP_SIZES).sum();
  private static final int STRIP_COUNT = STRIP_SIZES.length;

  // Number of LEDs for mirror annimations. This is int math, so it'll drop the remainder.
  private static final int MIRROR_LENGTH = LED_COUNT / 2;

  private final AtomicReference<Consumer<LEDStrips>> ledUpdateConsumer = new AtomicReference<Consumer<LEDStrips>>(null);
  private final Notifier ledNotifier;
  private final LEDStripMethods ledStripMethods = new LEDStripMethods();

  private final AddressableLED leds = new AddressableLED(DEVICE_ID);
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
    int index = 0;
    if (0 == stripId) {
      // Strip ID 0 is along the front, right to left
      // These LEDS are at the end of the string, in the same direction
      index = STRIP_SIZES[0] + STRIP_SIZES[1] + STRIP_SIZES[2];
      index += ledId;
    }
    if (1 == stripId) {
      // Strip ID 1 is along the left side, front to back
      // These LEDs are at the start of the string, in the same direction
      index = ledId;
    }
    if (2 == stripId) {
      // Strip ID 2 is along the right side, front to back
      // These LEDs are in the third segment of the string, in reverse order
      index = STRIP_SIZES[0] + STRIP_SIZES[1] + STRIP_SIZES[2] - 1;
      index -= ledId;
    }
    if (3 == stripId) {
      // StripID 3 is along the back side, right to left
      // These LEDs are in the second segment of the string, in reverse order
      index = STRIP_SIZES[0] + STRIP_SIZES[1] - 1;
      index -= ledId;
    }
    return index;
  }

  private int calculateMirrorIndex(int stripId, int ledId) {
    int index = 0;
    if (stripId == 0) {
      // Strip 0 runs along the left side of the robot, front to back
      if (ledId < STRIP_SIZES[3] / 2) {
        // Segment across the front left of robot (right to left)
        index = STRIP_SIZES[0] + STRIP_SIZES[1] + STRIP_SIZES[2] + (STRIP_SIZES[3] / 2) + 1;
        index += ledId;
      } else {
        // Segment across the left side of the robot (front to back), and across the back left (left to right)
        index = ledId - STRIP_SIZES[3] / 2;
      }
    } else {
      // Strip 1 runs along the right side of the robot, front to back
      index = STRIP_SIZES[0] + STRIP_SIZES[1] + STRIP_SIZES[2] + (STRIP_SIZES[3] / 2) - 1;
      index -= ledId;
    }

    return index;
  }

  /**
   * Implementation of {@link LEDStrips} for the LEDs managed by this subsystem
   */
  private class LEDStripMethods implements LEDStrips {
    
    @Override
    public void setLED(int stripId, int ledId, Color color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    @Override
    public void setLED(int stripId, int ledId, Color8Bit color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    @Override
    public void setMirrorLED(int ledId, Color color) {
      buffer.setLED(calculateMirrorIndex(0, ledId), color);
      buffer.setLED(calculateMirrorIndex(1, ledId), color);      
    }

    @Override
    public void setHSV(int stripId, int ledId, int h, int s, int v) {
      buffer.setHSV(calculateUpdateIndex(stripId, ledId), h, s, v);
    }

    @Override
    public void setMirrorHSV(int ledId, int h, int s, int v) {
      buffer.setHSV(calculateMirrorIndex(0, ledId), h, s, v);
      buffer.setHSV(calculateMirrorIndex(1, ledId), h, s, v);
    }

    @Override
    public void setRGB(int stripId, int ledId, int r, int g, int b) {
      buffer.setRGB(calculateUpdateIndex(stripId, ledId), r, g, b);
    }
    
    @Override
    public void refresh() {
      leds.setData(buffer);
    }

    @Override
    public void setAll(Color color) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setLED(i, color);
      }
      refresh();
    }

    @Override
    public void setAll(int r, int g, int b) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setRGB(i, r, g, b);
      }
      refresh();
    }

    @Override
    public void setLEDSegments(Color color, boolean... segmentValues) {
      // Only lights up the side strips, strips 1 and 2. Front and back stay off.
      // Light up the segments
      int ledsPerStatus = getStripSize(1) / segmentValues.length;
      for(int stripId = 1; stripId < LEDSubsystem.STRIP_COUNT; stripId ++) {
        int ledIndex = 0;
        for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
          for(;ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
            setLED(stripId, ledIndex, segmentValues[segmentId] ? color : Color.kBlack);
          }
        }
      }
      // Turn off the front and back strips (0 and 3)
      for (int stripId = 0; stripId < STRIP_COUNT; stripId += 3) {
        for (int ledId = 0; ledId < getStripSize(stripId); ledId++) {
          setLED(stripId, ledId, kBlack);
        }
      }
      refresh();
    }

    @Override
    public int getStripCount() {
      return STRIP_COUNT;
    }

    @Override
    public int getStripSize(int stripId) {
      switch (stripId) {
        case 0:
          return STRIP_SIZES[3];
        case 1:
          return STRIP_SIZES[0];
        case 2:
          return STRIP_SIZES[2];
        case 3:
          return STRIP_SIZES[1];
        default:
          return 0;
      }
    }

    @Override
    public int getMirrorStripSize() {
      return MIRROR_LENGTH;
    }
  }

}
