package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");
  private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
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

    fieldTypePub.set("Field2d");
    fieldPub.set(new double[] {
        dashboardPose.getX(),
        dashboardPose.getY(),
        dashboardPose.getRotation().getDegrees()
    });

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
    lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
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

    // Publish module states in AdvantageScope format
    var moduleStates = new double[state.ModuleStates.length * 2];
    for (int i = 0; i < state.ModuleStates.length; i++) {
      var moduleState = state.ModuleStates[i];
      moduleStates[i * 2] = moduleState.angle.getDegrees();
      moduleStates[i * 2 + 1] = moduleState.speedMetersPerSecond;
    }
    moduleStatePublisher.set(moduleStates);

  }
}
