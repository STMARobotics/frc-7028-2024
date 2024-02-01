package frc.robot.color;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Immutable value object to hold color and proximity data from a color sensor
 */
public class ColorSensorValues {
  /** Color from color sensor */
  public final Color color;

  /** Proximity from color sensor */
  public final int proximity;

  /**
   * Constructs a ColorSensorValues object
   * @param color Color from color sensor
   * @param proximity Proximity from color sensor
   */
  public ColorSensorValues(Color color, int proximity) {
    this.color = color;
    this.proximity = proximity;
  }
}
