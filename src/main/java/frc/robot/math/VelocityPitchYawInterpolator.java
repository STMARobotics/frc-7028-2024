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
 * WARNING: An instance of VelocityAngleInterpolator returns one mutable
 * ShootingSettings object from calculate.
 * Calls to calculate() will mutate the object returned from previous calls to
 * calculate(). This should be fine in
 * normal single-threaded robot code, but something to watch out for.
 */
public class VelocityPitchYawInterpolator {

  private final InterpolatingDoubleTreeMap distanceVelocityMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distancePitchMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceYawMap = new InterpolatingDoubleTreeMap();

  private final ShootingSettingsYaw shootingSettingsYaw = new ShootingSettingsYaw();

  /**
   * Constructor that takes a list of settings that will be loaded to the table.
   * 
   * @param settingsList list of settings
   */
  public VelocityPitchYawInterpolator(List<ShootingSettingsYaw> settingsList) {
    for (ShootingSettingsYaw settings : settingsList) {
      var distance = settings.distance.in(Meters);
      distanceVelocityMap.put(distance, settings.velocity.in(RotationsPerSecond));
      distancePitchMap.put(distance, settings.pitch.in(Radians));
      distanceYawMap.put(distance, settings.yaw.in(Radians));
    }
  }

  /**
   * Calculates the shooter velocity and pitch by interpolating based on the
   * distance.
   * 
   * @param distance distance to the target
   * @return shooter settings. The same instance of ShooterSettings is mutated and
   *         returned for every call.
   */
  public ShootingSettingsYaw calculate(double distance) {
    shootingSettingsYaw.distance.mut_replace(distance, Meters);
    shootingSettingsYaw.pitch.mut_replace(distancePitchMap.get(distance), Radians);
    shootingSettingsYaw.velocity.mut_replace(distanceVelocityMap.get(distance), RotationsPerSecond);
    shootingSettingsYaw.yaw.mut_replace(distanceYawMap.get(distance), Radians);
    return shootingSettingsYaw;
  }

  /**
   * Shooter settings
   */
  public static class ShootingSettingsYaw {
    private final MutableMeasure<Distance> distance = MutableMeasure.zero(Meters);
    private final MutableMeasure<Velocity<Angle>> velocity = MutableMeasure.zero(RotationsPerSecond);
    private final MutableMeasure<Angle> pitch = MutableMeasure.zero(Radians);
    private final MutableMeasure<Angle> yaw = MutableMeasure.zero(Radians);

    /**
     * Sets the distance
     * 
     * @param distance distance
     * @return this
     */
    public ShootingSettingsYaw distance(Measure<Distance> distance) {
      this.distance.mut_replace(distance);
      return this;
    }

    /**
     * Sets the pitch
     * 
     * @param pitch shooter pitch
     * @return this
     */
    public ShootingSettingsYaw pitch(Measure<Angle> pitch) {
      this.pitch.mut_replace(pitch);
      return this;
    }

    /**
     * Sets the yaw
     * 
     * @param pitch shooter pitch
     * @return this
     */
    public ShootingSettingsYaw yaw(Measure<Angle> yaw) {
      this.pitch.mut_replace(yaw);
      return this;
    }

    /**
     * Sets the shooter velocity
     * 
     * @param velocity velocity
     * @return this
     */
    public ShootingSettingsYaw velocity(Measure<Velocity<Angle>> velocity) {
      this.velocity.mut_replace(velocity);
      return this;
    }

    /**
     * Gets the shooter pitch
     * 
     * @return pitch
     */
    public Measure<Angle> getPitch() {
      return pitch;
    }

    /**
     * Gets the shooter yaw
     * 
     * @return pitch
     */
    public Measure<Angle> getYaw() {
      return yaw;
    }

    /**
     * Gets the distance to the target
     * 
     * @return distance
     */
    public Measure<Distance> getDistance() {
      return distance;
    }

    /**
     * Gets the shooter velocity
     * 
     * @return shooter velocity
     */
    public Measure<Velocity<Angle>> getVelocity() {
      return velocity;
    }

  }

}
