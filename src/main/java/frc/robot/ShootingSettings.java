package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Shooter settings */
public class ShootingSettings {
  private final AngularVelocity velocity;
  private final Angle pitch;

  /**
   * Constructor with velocity and pitch
   *
   * @param velocityRPS velocity in rotations per second
   * @param pitchDegrees pitch in degrees
   */
  public ShootingSettings(double velocityRPS, double pitchDegrees) {
    velocity = RotationsPerSecond.of(velocityRPS);
    pitch = Degrees.of(pitchDegrees);
  }

  /**
   * Gets the shooter pitch
   *
   * @return pitch
   */
  public Angle getPitch() {
    return pitch;
  }

  /**
   * Gets the shooter velocity
   *
   * @return shooter velocity
   */
  public AngularVelocity getVelocity() {
    return velocity;
  }

  /**
   * Interpolates between this and another ShootingSettings.
   *
   * @param endValue the end value
   * @param t the interpolation parameter [0, 1]
   * @return interpolated ShootingSettings
   */
  public ShootingSettings interpolate(ShootingSettings endValue, double t) {
    ShootingSettings result = new ShootingSettings(
        MathUtil.interpolate(velocity.in(RotationsPerSecond), endValue.velocity.in(RotationsPerSecond), t),
        MathUtil.interpolate(pitch.in(Radians), endValue.pitch.in(Radians), t));
    return result;
  }
}