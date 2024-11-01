package frc.robot.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import java.util.List;

/**
 * Table that will interpolate velocity and pitch based on distance. WARNING: An instance of
 * VelocityAngleInterpolator returns one mutable ShootingSettings object from calculate. Calls to
 * calculate() will mutate the object returned from previous calls to calculate(). This should be
 * fine in normal single-threaded robot code, but something to watch out for.
 */
public class VelocityPitchInterpolator {

	private final InterpolatingDoubleTreeMap distanceVelocityMap = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap distancePitchMap = new InterpolatingDoubleTreeMap();

	private final ShootingSettings shootingSettings = new ShootingSettings();

	/**
	 * Constructor that takes a list of settings that will be loaded to the table.
	 *
	 * @param settingsList list of settings
	 */
	public VelocityPitchInterpolator(List<ShootingSettings> settingsList) {
		for (ShootingSettings settings : settingsList) {
			var distance = settings.distance.in(Meters);
			distanceVelocityMap.put(distance, settings.velocity.in(RotationsPerSecond));
			distancePitchMap.put(distance, settings.pitch.in(Radians));
		}
	}

	/**
	 * Calculates the shooter velocity and pitch by interpolating based on the distance.
	 *
	 * @param distance distance to the target
	 * @return shooter settings. The same instance of ShooterSettings is mutated and returned for
	 *         every call.
	 */
	public ShootingSettings calculate(double distance) {
		shootingSettings.distance.mut_replace(distance, Meters);
		shootingSettings.pitch.mut_replace(distancePitchMap.get(distance), Radians);
		shootingSettings.velocity.mut_replace(distanceVelocityMap.get(distance), RotationsPerSecond);
		return shootingSettings;
	}

	/** Shooter settings */
	public static class ShootingSettings {
		private final MutDistance distance = Meters.mutable(0);
		private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);
		private final MutAngle pitch = Radians.mutable(0);

		/**
		 * Sets the distance
		 *
		 * @param distance distance
		 * @return this
		 */
		public ShootingSettings distance(Distance distance) {
			this.distance.mut_replace(distance);
			return this;
		}

		/**
		 * Sets the pitch
		 *
		 * @param pitch shooter pitch
		 * @return this
		 */
		public ShootingSettings pitch(Angle pitch) {
			this.pitch.mut_replace(pitch);
			return this;
		}

		/**
		 * Sets the shooter velocity
		 *
		 * @param velocity velocity
		 * @return this
		 */
		public ShootingSettings velocity(AngularVelocity velocity) {
			this.velocity.mut_replace(velocity);
			return this;
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
		 * Gets the distance to the target
		 *
		 * @return distance
		 */
		public Distance getDistance() {
			return distance;
		}

		/**
		 * Gets the shooter velocity
		 *
		 * @return shooter velocity
		 */
		public AngularVelocity getVelocity() {
			return velocity;
		}
	}
}
