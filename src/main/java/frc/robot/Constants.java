package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.PI;

import java.util.List;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.VelocityPitchInterpolator;
import frc.robot.math.VelocityPitchInterpolator.ShootingSettings;;

public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";
  
  public static final class DrivetrainConstants {
    public static final Measure<Distance> TRACKWIDTH = Inches.of(20.5);
    public static final Measure<Distance> WHEELBASE = Inches.of(21.125);

    // Theoretical free speed of L3 Kraken with FOC (Falcon 500 is faster)
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = FeetPerSecond.of(16.5);
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY
        .in(MetersPerSecond) /
        Math.hypot(TRACKWIDTH.in(Meters) / 2.0, WHEELBASE.in(Meters) / 2.0));
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(PI * 4 * 0.8);

    public static final MountPoseConfigs PIGEON_MOUNT_POSE_CONFIG = new MountPoseConfigs()
        .withMountPosePitch(9.096435546875)
        .withMountPoseRoll(-154.05722045898438)
        .withMountPoseYaw(6.09066915512085);
  }

  public static final class TeleopDriveConstants {

    public static final double TRANSLATION_DEADBAND = 0.05;
    public static final double ROTATIONAL_DEADBAND = 0.03;

    public static final Measure<Velocity<Velocity<Distance>>> TRANSLATION_RATE_LIMIT = MetersPerSecondPerSecond.of(20.0);
    public static final Measure<Velocity<Velocity<Angle>>> ROTATION_RATE_LIMIT =
        RadiansPerSecond.per(Second).of(8.0 * PI);

  }

  public static class AutoDriveConstants {

    public static final double THETA_kP = 4.0;
    public static final double THETA_kI = 0.001;
    public static final double THETA_kD = 0.0;

    public static final double TRANSLATION_kP = 5.0;
    public static final double TRANSLATION_kI = 0.0;
    public static final double TRANSLATION_kD = 0.0;

  }

  public static class VisionConstants {

    /**
     * Array of PhotonVision camera names. The values here match ROBOT_TO_CAMERA_TRANSFORMS for the camera's location.
    */
    public static final String[] APRILTAG_CAMERA_NAMES = {"Right", "Left"};

    /**
     * Physical location of the apriltag cameras on the robot, relative to the center of the robot.
     * The values here math APRILTAG_CAMERA_NAMES for the camera's name.
     */
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
        new Transform3d(
            new Translation3d(inchesToMeters(11.227), inchesToMeters(-10.446), inchesToMeters(8.238)),
            new Rotation3d(0, degreesToRadians(-25), degreesToRadians(-90))),
        new Transform3d(
            new Translation3d(inchesToMeters(11.227), inchesToMeters(10.446), inchesToMeters(8.238)),
            new Rotation3d(0, degreesToRadians(-25), degreesToRadians(90)))
      };

    public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
    public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when multiple tags are
     * in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final Measure<Distance> SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(5.0);

    // These Standard Deviations can be increased to "trust" vision measurements more. They are scaled based distance.
    /** Single tag standard deviation at 1-meter */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    /** Multitag standard deviation at 1-meter */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

  }

  public static class ShooterConstants {
    public static final int DEVICE_ID_TOP = 50;
    public static final int DEVICE_ID_BOTTOM = 51;

    public static final Measure<Current> PEAK_FORWARD_CURRENT = Amps.of(220);
    public static final Measure<Current> PEAK_REVERSE_CURRENT = PEAK_FORWARD_CURRENT.negate();
    public static final Measure<Current> SUPPLY_CURRENT_LIMIT = Amps.of(80);

    public static final SlotConfigs SLOT_CONFIG_TOP = new SlotConfigs()
        .withKP(10.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(6.5)
        .withKV(0.1);

    public static final SlotConfigs SLOT_CONFIG_BOTTOM = new SlotConfigs()
        .withKP(10.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(6.5)
        .withKV(0.101);
  
    public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
  
    public static final Measure<Velocity<Angle>> ERROR_TOLERANCE = RotationsPerSecond.of(1.5);
    public static final Measure<Velocity<Angle>> REVERSE_VELOCITY = RotationsPerSecond.of(-10.0);

    public static final Measure<Velocity<Angle>> AMP_TOP_VELOCITY = RotationsPerSecond.of(5.0);
    public static final Measure<Velocity<Angle>> AMP_BOTTOM_VELOCITY = RotationsPerSecond.of(26.0);
  }

  public static class IntakeConstants {
    public static final int DEVICE_ID = 42;
    public static final Measure<Current> PEAK_FORWARD_CURRENT = Amps.of(60);
    public static final Measure<Current> PEAK_REVERSE_CURRENT = PEAK_FORWARD_CURRENT.negate();
    public static final Measure<Current> SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final SlotConfigs SLOT_CONFIGS = new SlotConfigs()
        .withKP(34.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(16)
        .withKV(.32);
    public static final double SENSOR_TO_MECHANISM_RATIO = 3.0;
    public static final Measure<Velocity<Angle>> INTAKE_VELOCITY = RotationsPerSecond.of(17.5);
    public static final Measure<Velocity<Angle>> REVERSE_VELOCITY = RotationsPerSecond.of(-5.0);
  }

  public static class TurretConstants {
    public static final int DEVICE_ID_YAW_MOTOR = 45;
    public static final int DEVICE_ID_YAW_ENCODER = 46;
    public static final int DEVICE_ID_PITCH_MOTOR = 53;
    public static final int DEVICE_ID_PITCH_ENCODER = 54;
    public static final int DEVICE_ID_ROLLER_MOTOR = 41;
    public static final int DEVICE_ID_NOTE_SENSOR = 3;

    public static final Measure<Current> YAW_STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Measure<Current> YAW_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final Measure<Current> PITCH_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Measure<Current> PITCH_SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final Measure<Current> ROLLER_PEAK_FORWARD_CURRENT = Amps.of(60);
    public static final Measure<Current> ROLLER_PEAK_REVERSE_CURRENT = ROLLER_PEAK_FORWARD_CURRENT.negate();
    public static final Measure<Current> ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final Measure<Angle> YAW_MAGNETIC_OFFSET = Rotations.of(-0.096191); 
    public static final double YAW_ROTOR_TO_SENSOR_RATIO = (140 / 10) * 4.0;
    // NOTE: Yaw limits are set using turret encoder angles, so they're 180-degrees off from robot
    public static final Measure<Angle> YAW_LIMIT_FORWARD = Degrees.of(179.9);
    public static final Measure<Angle> YAW_LIMIT_REVERSE = Degrees.of(-179.9);

    // Range turret can shoot from without needing to turn the drivetrain
    public static final Measure<Angle> YAW_SHOOT_LIMIT_FORWARD = Rotations.of(0.323486);
    public static final Measure<Angle> YAW_SHOOT_LIMIT_REVERSE = Rotations.of(-0.641113);

    public static final SlotConfigs YAW_SLOT_CONFIGS = new SlotConfigs()
        .withKP(110)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.59)
        .withKV(0.5)
        .withKA(0.0);
    public static final MotionMagicConfigs YAW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(10.0)
        .withMotionMagicCruiseVelocity(2.0);

    public static Measure<Angle> PITCH_MAGNETIC_OFFSET = Rotations.of(0.480469);
    public static double PITCH_ROTOR_TO_SENSOR_RATIO = (348.0 / 20.0) * 9.0;
    public static Measure<Angle> PITCH_LIMIT_FORWARD = Rotations.of(0.181396);
    public static Measure<Angle> PITCH_LIMIT_REVERSE = Rotations.of(0.001);
    public static final SlotConfigs PITCH_SLOT_CONFIGS = new SlotConfigs()
        .withKP(130)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.4)
        .withKV(0.1)
        .withKA(0.0)
        .withKG(0.5)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    public static final MotionMagicConfigs PITCH_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(5.0)
        .withMotionMagicCruiseVelocity(1.5);

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs()
        .withKP(5.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(12);
    
    public static final Measure<Velocity<Angle>> INTAKE_VELOCITY = RotationsPerSecond.of(15);
    public static final Measure<Velocity<Angle>> EJECT_VELOCITY = RotationsPerSecond.of(-10);
    public static final Measure<Velocity<Angle>> SHOOT_VELOCITY = RotationsPerSecond.of(40);

    public static final Measure<Angle> INTAKE_YAW = Radians.of(PI);
    public static final Measure<Angle> INTAKE_PITCH = Radians.of(0.003);

    public static final Measure<Angle> AMP_YAW = Degrees.of(180);
    public static final Measure<Angle> AMP_PITCH = Degrees.of(37);

    public static final Measure<Angle> YAW_TOLERANCE = Degrees.of(1.0);
    public static final Measure<Angle> PITCH_TOLERANCE = Degrees.of(1);

    public static final Measure<Angle> INTAKE_YAW_TOLERANCE = Degrees.of(3.0);
    public static final Measure<Angle> INTAKE_PITCH_TOLERANCE = Degrees.of(3.0);

    public static final Measure<Angle> BABY_BIRD_YAW = Degrees.of(180.0);
    public static final Measure<Angle> BABY_BIRD_PITCH = Degrees.of(32.0);
    public static final Measure<Velocity<Angle>> BABY_BIRD_ROLLER_VELOCITY = RotationsPerSecond.of(-25);

    public static final Measure<Distance> NOTE_SENSOR_DISTANCE_THRESHOLD = Millimeter.of(150.0);

    /** Correction for note not launching perfectly straight from shooter wheels */
    public static final Measure<Angle> SHOOTING_YAW_CORRECTION = Degrees.of(1);

  }

  public static class LEDConstants {
    
    public static final int DEVICE_ID = 9;

    public static final Color NOTE_COLOR = Color.fromHSV(3, 255, 255);

  }

  public static class ShootingConstants {

    public static final double TARGET_OFFSET = inchesToMeters(4);
    public static final Translation2d SPEAKER_RED = new Translation2d(inchesToMeters(646.73) - TARGET_OFFSET, inchesToMeters(218.42));
    public static final Translation2d SPEAKER_BLUE = new Translation2d(TARGET_OFFSET, inchesToMeters(218.42));

    public static final Measure<Time> SHOOT_TIME = Seconds.of(0.5);
    public static final Measure<Angle> AIM_TOLERANCE = Degrees.of(1.5);
    public static final Measure<Velocity<Distance>> ROBOT_SPEED_TOLERANCE = MetersPerSecond.of(0.05);
    public static final Measure<Velocity<Angle>> ROBOT_ROTATION_TOLERANCE = DegreesPerSecond.of(15.0);

    /** A constant used applied to estimate the note's time of flight */
    public static final double SHOOT_WHILE_MOVING_COEFFICIENT = 0.0005;

    /**
     * Margin inside the turret shooting limits to avoid getting to the edge and being unable to reach
     */
    public static final Measure<Angle> DRIVETRAIN_MARGIN = Degrees.of(10);

    // Forward and reverse targets for the drivetrain when the turret is out of range
    public static final Rotation2d DRIVETRAIN_YAW_LIMIT_FORWARD =
        new Rotation2d(TurretConstants.YAW_SHOOT_LIMIT_FORWARD.minus(DRIVETRAIN_MARGIN));
    public static final Rotation2d DRIVETRAIN_YAW_LIMIT_REVERSE =
        new Rotation2d(TurretConstants.YAW_SHOOT_LIMIT_REVERSE.plus(DRIVETRAIN_MARGIN));

    public static final VelocityPitchInterpolator SHOOTER_INTERPOLATOR = new VelocityPitchInterpolator(List.of(
      new ShootingSettings().distance(Meters.of(1.34)).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(35)),
      new ShootingSettings().distance(Meters.of(1.56)).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(33)),
      new ShootingSettings().distance(Meters.of(1.922)).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(26)),
      new ShootingSettings().distance(Meters.of(2.216)).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(22)),
      new ShootingSettings().distance(Meters.of(2.673)).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(17)),
      new ShootingSettings().distance(Meters.of(3.442)).velocity(RotationsPerSecond.of(65)).pitch(Degrees.of(9)),
      new ShootingSettings().distance(Meters.of(4.005)).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(5)),
      new ShootingSettings().distance(Meters.of(4.445)).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(3)),
      new ShootingSettings().distance(Meters.of(4.905)).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(1.25)),
      new ShootingSettings().distance(Meters.of(5.357)).velocity(RotationsPerSecond.of(72)).pitch(Degrees.of(0)),
      new ShootingSettings().distance(Meters.of(5.728)).velocity(RotationsPerSecond.of(67.25)).pitch(Degrees.of(0)),
      new ShootingSettings().distance(Meters.of(6.194)).velocity(RotationsPerSecond.of(64.5)).pitch(Degrees.of(0.0))
    ));

  }

}