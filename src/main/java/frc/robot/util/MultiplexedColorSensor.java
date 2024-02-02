package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * This is a wrapper for the REV Color Sensor V3, allowing multiple sensors to be
 * used with a multiplexer board (this was tested using the Adafruit TCA9548A).
 * 
 * This class only exposes a single {@link #getValues()} method to get both the color
 * and proximity. For our purposes, we only need these two values, and we want both of
 * them for every robot loop. Combining them into a single method allows us to minimize
 * the writes to switch ports on the multiplexer.
 */
public class MultiplexedColorSensor {
  // Static map of multiplexers by I2C port. This is static because it is shared by all color sensors
  // connected to the multiplexer
  private static Map<Port, I2C> multiplexers = new HashMap<>();
  
  // Change this if your multiplexer has a different address. This is the TCA9548A's default address.
  private static final int kMultiplexerAddress = 0x70;

  // I2C for the multiplexer
  private final I2C multiplexer;

  // The actual sensor
  private ColorSensorV3 sensor;
  
  // What port on the multiplexer the color sensor is plugged into.
  private final int port;

  /**
   * Create a multiplexed color sensor.
   * 
   * @param i2cPort - What port the multiplexer is plugged into.
   * @param port    - What port the color sensor is plugged into the multiplexer
   *                (commonly labeled SC3 and SD3 on the PCB, where 3 is the
   *                port)
   */
  public MultiplexedColorSensor(I2C.Port i2cPort, int port) {
    if (multiplexers.get(i2cPort) == null) {
      multiplexer = new I2C(i2cPort, kMultiplexerAddress);
      multiplexers.put(i2cPort, multiplexer);
    } else {
      multiplexer = multiplexers.get(i2cPort);
    }
    this.port = port;
    // Set the channel before constructing the ColorSensorV3 because the constructor initalizes the sensor
    setChannel();
    sensor = new ColorSensorV3(i2cPort);
  }

  /**
   * Sets the multiplexer to the correct port before using the color sensor.
   */
  private void setChannel() {
    multiplexer.write(kMultiplexerAddress, 1 << port);
  }

  /**
   * Get the color and proximity values from the color sensor
   * @return color and proximity values
   */
  public ColorSensorValues getValues() {
    setChannel();
    return new ColorSensorValues(sensor.getColor(), sensor.getProximity());
  }

}