package frc.robot.generated;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains = new Slot0Configs().withKP(100)
      .withKI(0)
      .withKD(0.05)
      .withKS(0.18)
      .withKV(1.5)
      .withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains = new Slot0Configs().withKP(2.5)
      .withKI(0)
      .withKD(0)
      .withKS(0.78)
      .withKV(0)
      .withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = Robot.isSimulation() ? ClosedLoopOutputType.Voltage
      : ClosedLoopOutputType.TorqueCurrentFOC;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 70.0;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  private static final LinearVelocity kSpeedAt12VoltsMps = Constants.DrivetrainConstants.MAX_VELOCITY;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.5714285714285716;

  private static final double kDriveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  private static final double kMk4iSteerGearRatio = 150.0 / 7.0;
  private static final double kMk4SteerGearRatio = 12.8;
  private static final double kWheelRadiusInches = 1.9135;

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  // These two constants are 7028 specific because of our mix of MK4 and MK4i modules
  private static final boolean kInvertFrontSteer = false;
  private static final boolean kInvertRearSteer = true;

  private static final String kCANbusName = "canivore";
  private static final int kPigeonId = 30;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;

  private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withPigeon2Id(kPigeonId)
      .withCANBusName(kCANbusName);

  private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(kDriveGearRatio)
      .withWheelRadius(kWheelRadiusInches)
      .withSlipCurrent(kSlipCurrentA)
      .withSteerMotorGains(steerGains)
      .withDriveMotorGains(driveGains)
      .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
      .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
      .withSpeedAt12Volts(kSpeedAt12VoltsMps)
      .withSteerInertia(kSteerInertia)
      .withDriveInertia(kDriveInertia)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
      .withCouplingGearRatio(kCoupleRatio);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 3;
  private static final int kFrontLeftSteerMotorId = 13;
  private static final int kFrontLeftEncoderId = 23;
  private static final double kFrontLeftEncoderOffset = -0.184814;

  private static final double kFrontLeftXPosInches = 7.5;
  private static final double kFrontLeftYPosInches = 10.25;

  // Front Right
  private static final int kFrontRightDriveMotorId = 1;
  private static final int kFrontRightSteerMotorId = 11;
  private static final int kFrontRightEncoderId = 21;
  private static final double kFrontRightEncoderOffset = -0.292968;

  private static final double kFrontRightXPosInches = 7.5;
  private static final double kFrontRightYPosInches = -10.25;

  // Back Left
  private static final int kBackLeftDriveMotorId = 2;
  private static final int kBackLeftSteerMotorId = 12;
  private static final int kBackLeftEncoderId = 22;
  private static final double kBackLeftEncoderOffset = 0.301758;

  private static final double kBackLeftXPosInches = -13.625;
  private static final double kBackLeftYPosInches = 10.875;

  // Back Right
  private static final int kBackRightDriveMotorId = 0;
  private static final int kBackRightSteerMotorId = 10;
  private static final int kBackRightEncoderId = 20;
  private static final double kBackRightEncoderOffset = -0.351807;

  private static final double kBackRightXPosInches = -13.625;
  private static final double kBackRightYPosInches = -10.875;

  private static final SwerveModuleConstants FrontLeft = ConstantCreator.withSteerMotorGearRatio(kMk4SteerGearRatio)
      .createModuleConstants(
          kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            inchesToMeters(kFrontLeftXPosInches),
            inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide,
            kInvertFrontSteer);
  private static final SwerveModuleConstants FrontRight = ConstantCreator.withSteerMotorGearRatio(kMk4SteerGearRatio)
      .createModuleConstants(
          kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            inchesToMeters(kFrontRightXPosInches),
            inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide,
            kInvertFrontSteer);
  private static final SwerveModuleConstants BackLeft = ConstantCreator.withSteerMotorGearRatio(kMk4iSteerGearRatio)
      .createModuleConstants(
          kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            inchesToMeters(kBackLeftXPosInches),
            inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide,
            kInvertRearSteer);
  private static final SwerveModuleConstants BackRight = ConstantCreator.withSteerMotorGearRatio(kMk4iSteerGearRatio)
      .createModuleConstants(
          kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            inchesToMeters(kBackRightXPosInches),
            inchesToMeters(kBackRightYPosInches),
            kInvertRightSide,
            kInvertRearSteer);

  public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(
      DrivetrainConstants,
      FrontLeft,
      FrontRight,
      BackLeft,
      BackRight);
}
