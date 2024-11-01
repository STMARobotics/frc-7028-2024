package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Interface for updating LED strips */
public interface LEDStrips {
  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   *
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param color color to set
   */
  void setLED(int stripId, int ledId, Color color);

  /**
   * Sets LED color in the buffer for an animation that's mirrored across the left and right of the
   * robot. Call {@link #refresh()} to push update to the LED strip
   *
   * @param ledId ID of the LED in the animation
   * @param color color to set
   */
  void setMirrorLED(int ledId, Color color);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   *
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param color color to set
   */
  void setLED(int stripId, int ledId, Color8Bit color);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   *
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param h hue [0-180)
   * @param s saturation [0-255]
   * @param v value [0-255]
   */
  void setHSV(int stripId, int ledId, int h, int s, int v);

  /**
   * Sets LED color in the buffer for an animation that's mirrored across the left and right of the
   * robot. Call {@link #refresh()} to push update to the LED strip
   *
   * @param ledId ID of the LED in the animation
   * @param h hue [0-180)
   * @param s saturation [0-255]
   * @param v value [0-255]
   */
  void setMirrorHSV(int ledId, int h, int s, int v);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   *
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param r red [0-255]
   * @param g green [0-255]
   * @param b blue [0-255]
   */
  void setRGB(int stripId, int ledId, int r, int g, int b);

  /**
   * Writes the buffer to the LED strip. Call this once after doing all your updates with the set
   * color methods.
   */
  void refresh();

  /**
   * Sets all of the LEDs to the given color. Automatically refreshes.
   *
   * @param color color to set
   */
  void setAll(Color color);

  /**
   * Sets all of the LEDs to the given color. Automatically refreshes
   *
   * @param r red
   * @param g green
   * @param b blue
   */
  void setAll(int r, int g, int b);

  /**
   * Update LED strips in segments. Useful for indicating ready state, for example. Automatically
   * refreshes.
   *
   * @param segmentValues array of booleans. The strip will be split into segments one segment for
   *          each element of the array.
   */
  void setLEDSegments(Color color, boolean... segmentValues);

  /**
   * Gets the total number of strips
   *
   * @return number of strips
   */
  int getStripCount();

  /**
   * Gets the number of LEDs per strip
   *
   * @param stripId which stripID to get the length of
   * @return number of LEDs per strip
   */
  int getStripSize(int stripId);

  /**
   * Gets the length of the logical strip for mirrored annimation. Used with {@link
   * #setMirrorHSV(int, int, int, int)} and {@link #setMirrorLED(int, Color)}
   *
   * @return the number of LEDs in the logical mirrored strips
   */
  int getMirrorStripSize();
}
