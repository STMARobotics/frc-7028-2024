package frc.robot.subsystems;


import java.util.concurrent.atomic.AtomicReference;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.util.ColorSensorValues;

/**
 * Runnable to use with the WPILib Notifier to update the color sensor in the background.
 * This class enables a color sensor to be read on a background thread to avoid overrunning
 * the main robot loop.
 */
public class ColorSensorReader implements Runnable {
  
  private final ColorSensorV3 sensor = new ColorSensorV3(Port.kMXP);
  
  // These are AtomicReferences to ensure safe memory access across threads
  private AtomicReference<ColorSensorValues> fullValues = new AtomicReference<>();

   /**
   * Updates the color sensor values. It is safe to call this on a daemon thread and
   * call the "get" methods from the main robot thread.
   */
  public void run(){
    fullValues.set(new ColorSensorValues(sensor.getColor(), sensor.getProximity()));
  }

  /**
   * Gets the most recent values from the full sensor
   * @return full sensor values
   */
  public ColorSensorValues getValues() {
    return fullValues.get();
  }

}