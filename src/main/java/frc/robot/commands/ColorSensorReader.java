package frc.robot.commands;


import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_INTAKE_SENSOR;
import static frc.robot.Constants.IndexerConstants.PORT_ID_SPACER_SENSOR;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.util.ColorSensorValues;
import frc.robot.util.MultiplexedColorSensor;

/**
 * Runnable to use with the WPILib Notifier to update the color sensors in the background.
 * This class enables color sensors to be read on a background thread to avoid overrunning
 * the main robot loop.
 */
public class ColorSensorReader implements Runnable {
  
  private final MultiplexedColorSensor intakeColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_INTAKE_SENSOR);
  private final MultiplexedColorSensor spacerColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_SPACER_SENSOR);
  private final MultiplexedColorSensor fullColorSensor = new MultiplexedColorSensor(Port.kMXP, PORT_ID_FULL_SENSOR);
  
  // These are AtomicReferences to ensure safe memory access across threads
  private AtomicReference<ColorSensorValues> intakeValues = new AtomicReference<>();
  private AtomicReference<ColorSensorValues> spacerValues = new AtomicReference<>();
  private AtomicReference<ColorSensorValues> fullValues = new AtomicReference<>();

   /**
   * Updates the color sensor values. It is safe to call this on a daemon thread and
   * call the "get" methods from the main robot thread.
   */
  public void run(){
    intakeValues.set(intakeColorSensor.getValues());
    spacerValues.set(spacerColorSensor.getValues());
    fullValues.set(fullColorSensor.getValues());
  }

  /**
   * Gets the most recent values for the intake sensor
   * @return intake sensor values
   */
  public ColorSensorValues getIntakeValues() {
    return intakeValues.get();
  }

  /**
   * Gets the most recent values from the spacer sensor
   * @return spacer sensor values
   */
  public ColorSensorValues getSpacerValues() {
    return spacerValues.get();
  }

  /**
   * Gets the most recent values from the full sensor
   * @return full sensor values
   */
  public ColorSensorValues getFullValues() {
    return fullValues.get();
  }

}