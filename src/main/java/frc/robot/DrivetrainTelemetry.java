package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class DrivetrainTelemetry {
  // Limit telemetry updates to prevent flooding network and Shuffleboard
  private static final double PUBLISH_FREQUENCY = 0.04;

  private final int logEntry;
  private final int odomEntry;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");

  private final DoubleArrayPublisher moduleStatePublisher = driveStats.getDoubleArrayTopic("Module States").publish();
  private final DoubleArrayPublisher moduleTargetsPublisher = driveStats.getDoubleArrayTopic("Module Targets")
      .publish();
  private final DoublePublisher periodPublisher = driveStats.getDoubleTopic("Period").publish();

  private final Timer frequencyTimer = new Timer();

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public DrivetrainTelemetry(double maxSpeed) {
    logEntry = DataLogManager.getLog().start("odometry", "double[]");
    odomEntry = DataLogManager.getLog().start("odom period", "double");
    frequencyTimer.start();
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    if (frequencyTimer.advanceIfElapsed(PUBLISH_FREQUENCY)) {
      /* Telemeterize the pose */
      Pose2d pose = state.Pose;

      DataLogManager.getLog()
          .appendDoubleArray(
              logEntry,
                new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() },
                (long) (Timer.getFPGATimestamp() * 1000000));
      DataLogManager.getLog()
          .appendDouble(odomEntry, state.OdometryPeriod, (long) (Timer.getFPGATimestamp() * 1000000));

      // Publish module states and targets
      publishModuleStates(state.ModuleStates, moduleStatePublisher);
      publishModuleStates(state.ModuleTargets, moduleTargetsPublisher);
      periodPublisher.accept(state.OdometryPeriod);
    }
  }

  /**
   * Publishes an array of SwerveModuleStates in an array in the format expected for AdvantageScope
   *
   * @param states swerve module states to publish
   * @param publisher NT publisher
   */
  private static void publishModuleStates(SwerveModuleState[] states, DoubleArrayPublisher publisher) {
    var moduleStates = new double[states.length * 2];
    for (int i = 0; i < states.length; i++) {
      var moduleState = states[i];
      moduleStates[i * 2] = moduleState.angle.getDegrees();
      moduleStates[i * 2 + 1] = moduleState.speedMetersPerSecond;
    }
    publisher.set(moduleStates);
  }
}
