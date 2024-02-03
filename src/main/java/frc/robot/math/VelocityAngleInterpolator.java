package frc.robot.math;

import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
/**
 * Table that will interpolate velocity, angle, and height based on distance.
 */
public class VelocityAngleInterpolator {

  private final InterpolatingDoubleTreeMap distanceVelocityMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceHeightMap = new InterpolatingDoubleTreeMap();

  /**
   * Constructor that takes a list of settings that will be loaded to the table.
   * @param settingsList list of settings
   */
  public VelocityAngleInterpolator(List<DonutShooterSettings> settingsList) {
    for(DonutShooterSettings settings : settingsList) {
      distanceVelocityMap.put(settings.distance, settings.velocity);
      distanceAngleMap.put(settings.distance, settings.angle);
      distanceHeightMap.put(settings.distance, settings.height);
    }
  }

  /**
   * Calculates the shooter velocity, angle, and height by interpolating based on the distance.
   * @param distance distance to the target
   * @return shooter settings
   */
  public DonutShooterSettings calculate(double distance) {
    return DonutShooterSettings.shooterSettings(
        distance, distanceVelocityMap.get(distance), distanceAngleMap.get(distance), distanceHeightMap.get(distance));
  }

  /**
   * Shooter settings
   */
  public static class DonutShooterSettings {
    public final double distance;
    public final double velocity;
    public final double angle;
    public final double height;

    /**
     * Constructor
     * @param distance distance from target
     * @param velocity velocity of the shooter
     * @param angle angle of the wrist
     * @param height height of the elevator
     */
    public DonutShooterSettings(double distance, double velocity, double angle, double height) {
      this.distance = distance;
      this.velocity = velocity;
      this.angle = angle;
      this.height = height;
    }

    /**
     * Static factory method to make it more concise to create an instance
     * @param distance distance from target
     * @param velocity velocity of the shooter
     * @param wristAngle angle of the wrist
     * @param height height of the elevator
     * @return new instace with the provided values
     */
    public static DonutShooterSettings shooterSettings(double distance, double velocity, double wristAngle, double height) {
      return new DonutShooterSettings(distance, velocity, wristAngle, height);
    }
  }

}