package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Interface for updating LED strips
 */
public interface LEDStrips {
  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param color color to set
   */
  public void setLED(int stripId, int ledId, Color color);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param color color to set
   */
  public void setLED(int stripId, int ledId, Color8Bit color);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param h hue [0-180)
   * @param s saturation [0-255]
   * @param v value [0-255]
   */
  public void setHSV(int stripId, int ledId, int h, int s, int v);

  /**
   * Sets an LED color in the buffer. Call {@link #refresh()} to push update to the LED strip.
   * @param stripId ID of the strip
   * @param ledId ID of the LED
   * @param r red [0-255]
   * @param g green [0-255]
   * @param b blue [0-255]
   */
  public void setRGB(int stripId, int ledId, int r, int g, int b);

  /**
   * Writes the buffer to the LED strip. Call this once after doing all your updates with the set color methods.
   */
  public void refresh();

  /**
   * Sets all of the LEDs to the given color. Automatically refreshes.
   * @param color color to set
   */
  public void setAll(Color color);

  /**
   * Sets all of the LEDs to the given color. Automatically refreshes
   * @param r red
   * @param g green
   * @param b blue
   */
  public void setAll(int r, int g, int b);

  /**
   * Update LED strips in segments. Useful for indicating ready state, for example. Automatically refreshes.
   * @param segmentValues array of booleans. The strip will be split into segments one segment for each element of 
   * the array.
   */
  public void setLEDSegments(Color color, boolean... segmentValues);

  /**
   * Gets the total number of strips
   * @return number of strips
   */
  public int getStripCount();

  /**
   * Gets the number of LEDs per strip
   * @param stripId which stripID to get the length of
   * @return number of LEDs per strip
   */
  public int getStripSize(int stripId);

  /**
   * Gets the length of the longest strip
   * @return length of the longest strip
   */
  public int getMaxStripSize();
}
