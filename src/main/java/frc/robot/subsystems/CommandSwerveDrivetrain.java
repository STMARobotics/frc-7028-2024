package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_MOUNT_POSE_CONFIG;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PhotonRunnable;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private final Thread photonThread = new Thread(new PhotonRunnable(APRILTAG_CAMERA_NAMES, ROBOT_TO_CAMERA_TRANSFORMS,
      this::addVisionMeasurement, () -> getState().Pose));

  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

  private final SysIdRoutine translationSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(translationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(3), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(steerCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine rotationSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(rotationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine slipSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(translationCharacterization.withVolts(volts)), null, this));

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    tareEverything();

    // Start PhotonVision thread
    photonThread.setName("PhotonVision");
    photonThread.setDaemon(true);
    photonThread.start();

    m_pigeon2.getConfigurator().apply(PIGEON_MOUNT_POSE_CONFIG);

    configNeutralMode(NeutralModeValue.Brake);

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

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
            driveBaseRadius,
            new ReplanningConfig(),
            0.004),
        () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  protected boolean checkIsOnCanFD(String name) {
    // Troubleshooting method CTRE provided to help determine why odometry is running at 100Hz
    boolean isFd = CANBus.isNetworkFD(name);
    int maxCount = 20; // Wait at most 2 seconds checking if we're FD
    while (!isFd && maxCount-- > 0) {
      Timer.delay(0.1);
      isFd = CANBus.isNetworkFD(name);
      System.err.println("isNetworkFD reported network is not FD!!");
    }
    System.out.println("Network is FD? " + isFd);
    return isFd;
  }

  @Override
  public void simulationPeriodic() {
    /* Assume 20ms update rate, get battery voltage from WPILib */
    updateSimState(0.02, RobotController.getBatteryVoltage());
    SmartDashboard.putString("Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
  }

  /**
   * Gets the current robot-oriented chassis speeds
   * 
   * @return robot oriented chassis speeds
   */
  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return getState().speeds;
  }

  /**
   * Gets the current field-oriented chassis speeds
   * 
   * @return field oriented chassis speeds
   */
  public ChassisSpeeds getCurrentFieldChassisSpeeds() {
    var state = getState();
    if (state == null || state.Pose == null) {
      return new ChassisSpeeds();
    }
    var robotAngle = state.Pose.getRotation();
    var chassisSpeeds = state.speeds;
    var fieldSpeeds = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        .rotateBy(robotAngle);
    return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  public Command sysIdDriveQuasiCommand(Direction direction) {
    return translationSysIdRoutine.quasistatic(direction).withName("SysId Drive Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdDriveDynamCommand(SysIdRoutine.Direction direction) {
    return translationSysIdRoutine.dynamic(direction).withName("SysId Drive Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdSteerQuasiCommand(Direction direction) {
    return steerSysIdRoutine.quasistatic(direction).withName("SysId Steer Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdSteerDynamCommand(SysIdRoutine.Direction direction) {
    return steerSysIdRoutine.dynamic(direction).withName("SysId Steer Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdRotationDynamCommand(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.dynamic(direction).withName("SysId Rotate Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdRotationQuasiCommand(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.quasistatic(direction).withName("SysId Rotate Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdDriveSlipCommand() {
    return slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withName("SysId Drive Slip")
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

}
