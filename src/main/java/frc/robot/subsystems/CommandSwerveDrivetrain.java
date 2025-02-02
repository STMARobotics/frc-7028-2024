package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.QuestNav;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
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
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null,
          SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism(
          (output) -> setControl(rotationCharacterization.withRotationalRate(output.in(Volts))),
          null,
          this));

  private final SysIdRoutine slipSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(translationCharacterization.withVolts(volts)), null, this));

  private final QuestNav questNav = new QuestNav();

  private final NetworkTable questTestTable = NetworkTableInstance.getDefault().getTable("questTest");
  private final StructPublisher<Pose2d> questPoseEntry = questTestTable.getStructTopic("questPose", Pose2d.struct)
      .publish();
  private final StructPublisher<Pose2d> robotPoseEntry = questTestTable.getStructTopic("robotPose", Pose2d.struct)
      .publish();

  private Notifier simNotifier = null;
  private double lastSimTime;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    tareEverything();

    configNeutralMode(NeutralModeValue.Brake);

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    try {
      var robotConfig = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
            this::resetPose, // Consumer for seeding pose against auto
            () -> getState().Speeds, // Supplier of current robot speeds
            (speeds, feedforwards) -> this.setControl(
                // Consumer of ChassisSpeeds to drive the robot
                autoRequest.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
            new PPHolonomicDriveController(
                new PIDConstants(
                    AutoDriveConstants.TRANSLATION_kP,
                    AutoDriveConstants.TRANSLATION_kI,
                    AutoDriveConstants.TRANSLATION_kD),
                new PIDConstants(
                    AutoDriveConstants.THETA_kP,
                    AutoDriveConstants.THETA_kI,
                    AutoDriveConstants.THETA_kD)),
            robotConfig,
            () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
            this); // Subsystem for requirements
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - lastSimTime;
      lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
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
    var chassisSpeeds = state.Speeds;
    var fieldSpeeds = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        .rotateBy(robotAngle);
    return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void resetPose(Pose2d pose) {
    questNav.resetPose(pose);
    super.resetPose(pose);
  }

  @Override
  public void periodic() {
    questNav.cleanUpQuestNavMessages();
    questPoseEntry.set(questNav.getQuestPose());
    robotPoseEntry.set(questNav.getRobotPose());
  }

  public Command sysIdDriveQuasiCommand(Direction direction) {
    return translationSysIdRoutine.quasistatic(direction)
        .withName("SysId Drive Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdDriveDynamCommand(SysIdRoutine.Direction direction) {
    return translationSysIdRoutine.dynamic(direction)
        .withName("SysId Drive Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdSteerQuasiCommand(Direction direction) {
    return steerSysIdRoutine.quasistatic(direction)
        .withName("SysId Steer Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdSteerDynamCommand(SysIdRoutine.Direction direction) {
    return steerSysIdRoutine.dynamic(direction)
        .withName("SysId Steer Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdRotationDynamCommand(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.dynamic(direction)
        .withName("SysId Rotate Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdRotationQuasiCommand(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.quasistatic(direction)
        .withName("SysId Rotate Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  public Command sysIdDriveSlipCommand() {
    return slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        .withName("SysId Drive Slip")
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   * <p>
   * Note that the vision measurement standard deviations passed into this method
   * will continue to apply to future measurements until a subsequent call to
   * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
   *          in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters,
          Utils.fpgaToCurrentTime(timestampSeconds),
          visionMeasurementStdDevs);
  }
}
