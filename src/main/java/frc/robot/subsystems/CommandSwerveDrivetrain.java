package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAME;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.PhotonRunnable;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final Thread photonThread = new Thread(new PhotonRunnable(APRILTAG_CAMERA_NAME, this::addVisionMeasurement));

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    tareEverything();
    
    // Start PhotonVision thread
    photonThread.setName("PhotonVision");
    photonThread.start();

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(AutoDriveConstants.TRANSLATION_kP, AutoDriveConstants.TRANSLATION_kI,
                AutoDriveConstants.TRANSLATION_kD),
            new PIDConstants(AutoDriveConstants.THETA_kP, AutoDriveConstants.THETA_kI, AutoDriveConstants.THETA_kD),
            DrivetrainConstants.MAX_VELOCITY.in(Units.MetersPerSecond),
            new Translation2d(DrivetrainConstants.WHEELBASE.in(Units.Meters) / 2.0,
                DrivetrainConstants.TRACKWIDTH.in(Units.Meters) / 2.0).getNorm(),
            new ReplanningConfig(),
            0.004),
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  @Override
  public void simulationPeriodic() {
    /* Assume 20ms update rate, get battery voltage from WPILib */
    updateSimState(0.02, RobotController.getBatteryVoltage());
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    sawTag = true;
    var visionPose2d = visionRobotPoseMeters;
    if (originPosition != kBlueAllianceWallRightSide) {
      visionPose2d = VisionConstants.flipAlliance(visionPose2d);
    }
    super.addVisionMeasurement(visionPose2d, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    sawTag = true;
    var visionPose2d = visionRobotPoseMeters;
    if (originPosition != kBlueAllianceWallRightSide) {
      visionPose2d = VisionConstants.flipAlliance(visionPose2d);
    }
    super.addVisionMeasurement(visionPose2d, timestampSeconds, visionMeasurementStdDevs);
  }
  
  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      m_stateLock.writeLock().lock();
      try {
        var newPose = VisionConstants.flipAlliance(getState().Pose);
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, newPose);
      } finally {
        m_stateLock.writeLock().unlock();
      }
    }
  }

      // Overriden with workaround for https://github.com/CrossTheRoadElec/Phoenix6-Examples/issues/27
      @Override
      public void seedFieldRelative(Pose2d location) {
          try {
              m_stateLock.writeLock().lock();
              m_cachedState.Pose = location; // Workaround: update cached state
              m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
          } finally {
              m_stateLock.writeLock().unlock();
          }
      }

}
