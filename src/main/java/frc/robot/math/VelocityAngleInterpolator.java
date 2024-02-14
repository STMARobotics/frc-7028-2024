package frc.robot.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

/**
 * Table that will interpolate velocity, angle, and height based on distance.
 * WARNING: An instance of VelocityAngleInterpolator returns one mutable ShootingSettings object from calculate.
 * Calls to calculate() will mutate the object returned from previous calls to calculate(). This should be fine in
 * normal single-threaded robot code, but something to watch out for.
 */
public class VelocityAngleInterpolator {

  private final InterpolatingDoubleTreeMap distanceVelocityMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceHeightMap = new InterpolatingDoubleTreeMap();

  private final ShootingSettings shootingSettings = new ShootingSettings();

  /**
   * Constructor that takes a list of settings that will be loaded to the table.
   * @param settingsList list of settings
   */
  public VelocityAngleInterpolator(List<ShootingSettings> settingsList) {
    for(ShootingSettings settings : settingsList) {
      var distance = settings.distance.in(Meters);
      distanceVelocityMap.put(distance, settings.velocity.in(RotationsPerSecond));
      distanceAngleMap.put(distance, settings.angle.in(Radians));
      distanceHeightMap.put(distance, settings.height.in(Meters));
    }
  }

  /**
   * Calculates the shooter velocity, angle, and height by interpolating based on the distance.
   * @param distance distance to the target
   * @return shooter settings. The same instance of ShooterSettings is mutated and returned for every call.
   */
  public ShootingSettings calculate(double distance) {
    return shootingSettings
      .withDistance(distance)
      .withAngle(distanceAngleMap.get(distance))
      .withHeight(distanceHeightMap.get(distance))
      .withVelocity(distanceVelocityMap.get(distance));
  }

  /**
   * Shooter settings
   */
  public static class ShootingSettings {
    private final MutableMeasure<Distance> distance = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);
    private final MutableMeasure<Angle> angle = MutableMeasure.zero(Radians);
    private final MutableMeasure<Distance> height = MutableMeasure.zero(Meters);

    /**
     * Factory method that works well with chaining.
     * <pre>
     * distance(Meters.of(1.0)).velocity(RotationsPerSecond.of(40)).angle(Rotations.of(0.35)).height(Meters.of(0.1))
     * </pre>
     * @param distance distance
     * @return new shooter settings with the given distance
     */
    public static ShootingSettings distance(Measure<Distance> distance) {
      return new ShootingSettings().withDistance(distance.in(Meters));
    }

    /**
     * Sets the angle
     * @param angle shooter angle
     * @return this
     */
    public ShootingSettings angle(Measure<Angle> angle) {
      this.angle.mut_replace(angle);
      return this;
    }

    /**
     * Sets the elevator height
     * @param height elevator height
     * @return this
     */
    public ShootingSettings height(Measure<Distance> height) {
      this.height.mut_replace(height);
      return this;
    }

    /**
     * Sets the shooter velocity
     * @param velocity velocity
     * @return this
     */
    public ShootingSettings velocity(Measure<Velocity<Angle>> velocity) {
      this.velocity.mut_replace(velocity);
      return this;
    }

    /**
     * Gets the shooter angle
     * @return angle
     */
    public Measure<Angle> getAngle() {
      return angle;
    }

    /**
     * Gets the distance to the target
     * @return distance
     */
    public Measure<Distance> getDistance() {
      return distance;
    }

    /**
     * Gets the elevator height
     * @return elevator height
     */
    public Measure<Distance> getHeight() {
      return height;
    }

    /**
     * Gets the shooter velocity
     * @return shooter velocity
     */
    public Measure<Velocity<Angle>> getVelocity() {
      return velocity;
    }

    private ShootingSettings withAngle(double angleRadians) {
      this.angle.mut_replace(angleRadians, Radians);
      return this;
    }

    private ShootingSettings withDistance(double distanceMeters) {
      this.distance.mut_replace(distanceMeters, Meters);
      return this;
    }

    private ShootingSettings withHeight(double heightMeters) {
      this.height.mut_replace(heightMeters, Meters);
      return this;
    }

    private ShootingSettings withVelocity(double velocityRPS) {
      this.velocity.mut_replace(velocityRPS, RotationsPerSecond);
      return this;
    }

  }

}
