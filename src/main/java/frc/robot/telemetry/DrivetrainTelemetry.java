package frc.robot.telemetry;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DrivetrainTelemetry {
  // Limit telemetry updates to prevent flooding network and Shuffleboard
  private static final double PUBLISH_FREQUENCY = 0.04;

  private final int logEntry;
  private final int odomEntry;

  private final ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final Field2d field2d = new Field2d();

  private final GenericEntry drivetrainSpeedEntry;
  private final GenericEntry drivetrainVelocityXEntry;
  private final GenericEntry drivetrainVelocityYEntry;
  private final GenericEntry periodEntry;
  private final GenericEntry readablePoseEntry;

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");

  private final DoubleArrayPublisher moduleStatePublisher = 
      driveStats.getDoubleArrayTopic("Module States").publish();
  private final DoubleArrayPublisher moduleTargetsPublisher =
      driveStats.getDoubleArrayTopic("Module Targets").publish();
  
  private final Timer frequencyTimer = new Timer();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   * 
   * @param maxSpeed
   *          Maximum speed in meters per second
   */
  public DrivetrainTelemetry(double maxSpeed) {
    drivetrainTab.add(field2d).withPosition(0, 0).withSize(5, 3);

    readablePoseEntry = drivetrainTab.add("Pose", "").withWidget(BuiltInWidgets.kTextView)
        .withSize(2, 1)
        .withPosition(0, 3).getEntry();

    var drivetrainVelocityGrid = drivetrainTab.getLayout("Velocity", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
          .withSize(2, 3)
          .withPosition(5, 0);

    drivetrainSpeedEntry = drivetrainVelocityGrid.add("Speed", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", Math.ceil(maxSpeed))).getEntry();
    drivetrainVelocityXEntry = drivetrainVelocityGrid.add("Velocity X", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", -Math.ceil(maxSpeed), "Max", Math.ceil(maxSpeed))).getEntry();
    drivetrainVelocityYEntry = drivetrainVelocityGrid.add("Velocity Y", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", -Math.ceil(maxSpeed), "Max", Math.ceil(maxSpeed))).getEntry();
    
    periodEntry = drivetrainTab.add("Update Frequency (Hz)", 0).withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(7, 0) .getEntry();
    
    logEntry = DataLogManager.getLog().start("odometry", "double[]");
    odomEntry = DataLogManager.getLog().start("odom period", "double");
    frequencyTimer.start();
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    if (frequencyTimer.advanceIfElapsed(PUBLISH_FREQUENCY)) {
      /* Telemeterize the pose */
      Pose2d pose = state.Pose;
      field2d.setRobotPose(pose);

      readablePoseEntry.setString(String.format("(%.3f, %.3f) %.2f rad", 
          pose.getX(), pose.getY(), pose.getRotation().getRadians()));

      /* Telemeterize the robot's general speeds */
      double currentTime = Utils.getCurrentTimeSeconds();
      double diffTime = currentTime - lastTime;
      lastTime = currentTime;
      Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
      lastPose = pose;

      Translation2d velocities = distanceDiff.div(diffTime);

      drivetrainSpeedEntry.setDouble(velocities.getNorm());
      drivetrainVelocityXEntry.setDouble(velocities.getX());
      drivetrainVelocityYEntry.setDouble(velocities.getY());

      periodEntry.setDouble(1.0 / state.OdometryPeriod);

      DataLogManager.getLog().appendDoubleArray(logEntry,
          new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() },
          (long) (Timer.getFPGATimestamp() * 1000000));
      DataLogManager.getLog().appendDouble(odomEntry, state.OdometryPeriod, (long) (Timer.getFPGATimestamp() * 1000000));

      // Publish module states and targets
      publishModuleStates(state.ModuleStates, moduleStatePublisher);
      publishModuleStates(state.ModuleTargets, moduleTargetsPublisher);
    }
  }

  /**
   * Publishes an array of SwerveModuleStates in an array in the format expected for AdvantageScope
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
