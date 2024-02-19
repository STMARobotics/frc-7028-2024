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
 * Table that will interpolate velocity and pitch based on distance.
 * WARNING: An instance of VelocityAngleInterpolator returns one mutable ShootingSettings object from calculate.
 * Calls to calculate() will mutate the object returned from previous calls to calculate(). This should be fine in
 * normal single-threaded robot code, but something to watch out for.
 */
public class VelocityPitchInterpolator {

  private final InterpolatingDoubleTreeMap distanceVelocityMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distancePitchMap = new InterpolatingDoubleTreeMap();

  private final ShootingSettings shootingSettings = new ShootingSettings();

  /**
   * Constructor that takes a list of settings that will be loaded to the table.
   * @param settingsList list of settings
   */
  public VelocityPitchInterpolator(List<ShootingSettings> settingsList) {
    for(ShootingSettings settings : settingsList) {
      var distance = settings.distance.in(Meters);
      distanceVelocityMap.put(distance, settings.velocity.in(RotationsPerSecond));
      distancePitchMap.put(distance, settings.pitch.in(Radians));
    }
  }

  /**
   * Calculates the shooter velocity and pitch by interpolating based on the distance.
   * @param distance distance to the target
   * @return shooter settings. The same instance of ShooterSettings is mutated and returned for every call.
   */
  public ShootingSettings calculate(double distance) {
    shootingSettings.distance.mut_replace(distance, Meters);
    shootingSettings.pitch.mut_replace(distancePitchMap.get(distance), Radians);
    shootingSettings.velocity.mut_replace(distanceVelocityMap.get(distance), RotationsPerSecond);
    return shootingSettings;
  }

  /**
   * Shooter settings
   */
  public static class ShootingSettings {
    private final MutableMeasure<Distance> distance = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);
    private final MutableMeasure<Angle> pitch = MutableMeasure.zero(Radians);

    /**
     * Sets the distance
     * @param distance distance
     * @return this
     */
    public ShootingSettings distance(Measure<Distance> distance) {
      this.distance.mut_replace(distance);
      return this;
    }

    /**
     * Sets the pitch
     * @param pitch shooter pitch
     * @return this
     */
    public ShootingSettings pitch(Measure<Angle> pitch) {
      this.pitch.mut_replace(pitch);
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
     * Gets the shooter pitch
     * @return pitch
     */
    public Measure<Angle> getPitch() {
      return pitch;
    }

    /**
     * Gets the distance to the target
     * @return distance
     */
    public Measure<Distance> getDistance() {
      return distance;
    }

    /**
     * Gets the shooter velocity
     * @return shooter velocity
     */
    public Measure<Velocity<Angle>> getVelocity() {
      return velocity;
    }

  }

}
