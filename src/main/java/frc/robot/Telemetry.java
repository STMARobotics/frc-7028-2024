package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.VisionConstants;

public class Telemetry {
  private final double maxSpeed;
  private final int logEntry;
  private final int odomEntry;

  private final ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final Field2d field2d = new Field2d();

  private final GenericEntry drivetrainSpeed;
  private final GenericEntry drivetrainVelocityX;
  private final GenericEntry drivetrainVelocityY;

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");
  private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
  };

  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
  };

  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
  };

  private final DoubleArrayPublisher moduleStatePublisher = driveStats.getDoubleArrayTopic("Module States").publish();
  private final DoubleArrayPublisher moduleTargetsPublisher = driveStats.getDoubleArrayTopic("Module Targets").publish();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();
  private Alliance alliance = Alliance.Blue;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   * 
   * @param maxSpeed
   *          Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    this.maxSpeed = maxSpeed;

    drivetrainTab.add(field2d).withPosition(0, 0).withSize(5, 3);

    var drivetrainVelocityGrid = drivetrainTab.getLayout("Velocity", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
          .withSize(2, 3)
          .withPosition(5, 0);

    drivetrainSpeed = drivetrainVelocityGrid.add("Speed", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", Math.ceil(maxSpeed))).getEntry();
    drivetrainVelocityX = drivetrainVelocityGrid.add("Velocity X", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", -Math.ceil(maxSpeed), "Max", Math.ceil(maxSpeed))).getEntry();
    drivetrainVelocityY = drivetrainVelocityGrid.add("Velocity Y", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", -Math.ceil(maxSpeed), "Max", Math.ceil(maxSpeed))).getEntry();
    
    logEntry = DataLogManager.getLog().start("odometry", "double[]");
    odomEntry = DataLogManager.getLog().start("odom period", "double");
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d pose = state.Pose;
    var dashboardPose = pose;
    if (alliance == Alliance.Red) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = VisionConstants.flipAlliance(dashboardPose);
    }

    field2d.setRobotPose(dashboardPose);

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
    lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    drivetrainSpeed.setDouble(velocities.getNorm());
    drivetrainVelocityX.setDouble(velocities.getX());
    drivetrainVelocityY.setDouble(velocities.getY());

    odomPeriod.set(1.0 / state.OdometryPeriod);

    /* Telemeterize the module's states */
    for (int i = 0; i < 4; ++i) {
      moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));

      SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
    }

    DataLogManager.getLog().appendDoubleArray(logEntry,
        new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() },
        (long) (Timer.getFPGATimestamp() * 1000000));
    DataLogManager.getLog().appendDouble(odomEntry, state.OdometryPeriod, (long) (Timer.getFPGATimestamp() * 1000000));

    // Publish module states and targets
    publishModuleStates(state.ModuleStates, moduleStatePublisher);
    publishModuleStates(state.ModuleTargets, moduleTargetsPublisher);
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
