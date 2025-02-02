package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Consumer that uses vision measurements. Usually implemented by the drivetrain to pass data to a pose estimator.
 */
@FunctionalInterface
public interface VisionConsumer {
  /**
   * Adds a vision measurement.
   * <p>
   * Note that the vision measurement standard deviations passed into this method
   * will continue to apply to future measurements.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form [x, y, theta]ᵀ, with
   *          units in meters and radians.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
}
