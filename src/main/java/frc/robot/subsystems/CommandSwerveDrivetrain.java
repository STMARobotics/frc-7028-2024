package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAME;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PhotonRunnable;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;
import frc.robot.subsystems.sysid.VoltageSwerveRequest;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final Thread photonThread = new Thread(new PhotonRunnable(APRILTAG_CAMERA_NAME, this::addVisionMeasurement));

  private VoltageSwerveRequest voltageRequest = new VoltageSwerveRequest();

  private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(voltageRequest.withDriveVoltage(volts)), null, this));

  private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(voltageRequest.withSteerVoltage(volts)), null, this));

  private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> setControl(voltageRequest.withDriveVoltage(volts)), null, this));

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
        () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void simulationPeriodic() {
    /* Assume 20ms update rate, get battery voltage from WPILib */
    updateSimState(0.02, RobotController.getBatteryVoltage());
    SmartDashboard.putString("Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command sysIdDriveQuasiCommand(Direction direction) {
    return m_driveSysIdRoutine.quasistatic(direction).withName("SysId Drive Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdDriveDynamCommand(SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutine.dynamic(direction).withName("SysId Drive Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdSteerQuasiCommand(Direction direction) {
    return m_steerSysIdRoutine.quasistatic(direction).withName("SysId Steer Quasistatic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdSteerDynamCommand(SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutine.dynamic(direction).withName("SysId Steer Dynamic " + direction)
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

  public Command sysIdDriveSlipCommand() {
    return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withName("SysId Drive Slip")
        .finallyDo(() -> this.setControl(new SwerveRequest.ApplyChassisSpeeds()));
  }

}