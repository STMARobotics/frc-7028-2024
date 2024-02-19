package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.math.VelocityAngleInterpolator.ShootingSettings.distance;
import static java.lang.Math.PI;

import java.util.List;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import frc.robot.math.VelocityAngleInterpolator;

public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";
  
  public static final class DrivetrainConstants {
    public static final Measure<Distance> TRACKWIDTH = Inches.of(18.75);
    public static final Measure<Distance> WHEELBASE = Inches.of(18.75);

    // Theoretical free speed of L3 Kraken with FOC (Falcon 500 is faster)
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = FeetPerSecond.of(16.5);
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY
        .in(MetersPerSecond) /
        Math.hypot(TRACKWIDTH.in(Meters) / 2.0, WHEELBASE.in(Meters) / 2.0));
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(PI * 4);

    public static final MountPoseConfigs PIGEON_MOUNT_POSE_CONFIG = new MountPoseConfigs()
        .withMountPosePitch(9.096435546875)
        .withMountPoseRoll(-154.05722045898438)
        .withMountPoseYaw(6.09066915512085);
  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final Measure<Velocity<Velocity<Distance>>> TRANSLATION_RATE_LIMIT = MetersPerSecondPerSecond.of(8.0);
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
            new Translation3d(inchesToMeters(7.678), inchesToMeters(-12.333), inchesToMeters(10.619)),
            new Rotation3d(0, degreesToRadians(-25), -PI)),
        new Transform3d(
            new Translation3d(inchesToMeters(7.678), inchesToMeters(12.333), inchesToMeters(10.619)),
            new Rotation3d(0, degreesToRadians(-25), PI))
      };

    public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
    public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when multiple tags are
     * in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

  }

  public static class ShooterConstants {
    public static final int DEVICE_ID_TOP = 50;
    public static final int DEVICE_ID_BOTTOM = 51;

    public static final SlotConfigs SHOOTER_VELOCITY_SLOT_CONFIG_TOP = new SlotConfigs()
        .withKP(12.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(6.5)
        .withKV(0.1);

    public static final SlotConfigs SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM = new SlotConfigs()
        .withKP(10.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(6.5)
        .withKV(0.101);
  
    public static final double SHOOTER_SENSOR_TO_MECHANISM_RATIO = 1.0;
  
    public static final Measure<Velocity<Angle>> SHOOTER_ERROR_TOLERANCE = RotationsPerSecond.of(5.0);
  }

  public static class IntakeConstants {
    public static final int DEVICE_ID_ROLLERS = 42;

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs()
        .withKP(8.5)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(16)
        .withKV(.08);
    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 1.0;
    public static final Measure<Velocity<Angle>> ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(50.0);
    public static final Measure<Velocity<Angle>> ROLLER_REVERSE_VELOCITY = RotationsPerSecond.of(-20.0);
  }

  public static class AmperConstants {
    public static final int DEVICE_ID_ROLLERS = 40;

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs()
        .withKP(4.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(22)
        .withKV(0.17);
    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 1.0;
    public static final Measure<Velocity<Angle>> ROLLER_LOAD_VELOCITY = RotationsPerSecond.of(-50.0);
    public static final Measure<Velocity<Angle>> ROLLER_SCORE_VELOCITY = RotationsPerSecond.of(-10.0);
    public static final Measure<Velocity<Angle>> ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(50.0);
  }

  public static class TurretConstants {
    public static int DEVICE_ID_YAW_MOTOR = 45;
    public static int DEVICE_ID_YAW_ENCODER = 46;
    public static int DEVICE_ID_PITCH_MOTOR = 53;
    public static int DEVICE_ID_PITCH_ENCODER = 54;
    public static int DEVICE_ID_ROLLER_MOTOR = 41;
    public static int DEVICE_ID_NOTE_SENSOR = 5;

    public static Measure<Angle> YAW_MAGNETIC_OFFSET = Rotations.of(-0.260742);
    public static double YAW_ROTOR_TO_SENSOR_RATIO = (140 / 10) * 4.0;
    public static Measure<Angle> YAW_LIMIT_FORWARD = Rotations.of(0.45);
    public static Measure<Angle> YAW_LIMIT_REVERSE = Rotations.of(-0.45);
    public static final SlotConfigs YAW_SLOT_CONFIGS = new SlotConfigs()
        .withKP(45)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.6)
        .withKV(8.0)
        .withKA(0.0);
    public static final MotionMagicConfigs YAW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(3)
        .withMotionMagicCruiseVelocity(.75);

    public static Measure<Angle> PITCH_MAGNETIC_OFFSET = Rotations.of(-0.025635);
    public static double PITCH_ROTOR_TO_SENSOR_RATIO = (348.0 / 20.0) * 5.0;
    public static Measure<Angle> PITCH_LIMIT_FORWARD = Rotations.of(0.115);
    public static Measure<Angle> PITCH_LIMIT_REVERSE = Rotations.of(0.004);
    public static final SlotConfigs PITCH_SLOT_CONFIGS = new SlotConfigs()
        .withKP(45)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.6)
        .withKV(8.0)
        .withKA(0.0)
        .withKG(0.45)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    public static final MotionMagicConfigs PITCH_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(3)
        .withMotionMagicCruiseVelocity(.75);

    public static final SlotConfigs ROLLER_VELOCITY_SLOT_CONFIGS = new SlotConfigs()
        .withKP(5.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(12);
    
    public static final Measure<Velocity<Angle>> LOAD_VELOCITY = RotationsPerSecond.of(25);
    public static final Measure<Velocity<Angle>> EJECT_VELOCITY = RotationsPerSecond.of(-10);
    public static final Measure<Velocity<Angle>> SHOOT_VELOCITY = RotationsPerSecond.of(30);

    public static final Measure<Angle> INTAKE_YAW_POSITION = Radians.of(PI);
    public static final Measure<Angle> INTAKE_PITCH_POSITION = Radians.of(0.0);

    public static final Measure<Angle> YAW_TOLERANCE = Degrees.of(2);
    public static final Measure<Angle> PITCH_TOLERANCE = Degrees.of(2);

  }

  public static class ElevatorConstants {
    public static int DEVICE_ID_MOTOR = 60;
    public static int DEVICE_PORT_TOP_LIMIT = 9;
    public static int DEVICE_PORT_BOTTOM_LIMIT = 8;

    public static Measure<Per<Distance, Angle>> METERS_PER_ROTATION = 
        Meters.per(Rotations).of((inchesToMeters(1.27) * PI) / 3.0 / 3.0);

    public static SlotConfigs SLOT_CONFIGS = new SlotConfigs()
        .withKP(0.01)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0)
        .withKG(0.01)
        .withGravityType(GravityTypeValue.Elevator_Static);
    
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(0.01)
        .withMotionMagicCruiseVelocity(0.01);
    
    public static final Measure<Distance> TOP_LIMIT = Meters.of(0.42);
    public static final Measure<Distance> BOTTOM_LIMIT = Meters.zero();
  }

  public static class ShootingConstants {

    public static final Translation2d SPEAKER_RED = new Translation2d(inchesToMeters(652.73), inchesToMeters(218.42));
    public static final Translation2d SPEAKER_BLUE = new Translation2d(0.0, inchesToMeters(218.42));

    public static final Measure<Time> SHOOT_TIME = Seconds.of(2.0);
    public static final Measure<Angle> AIM_TOLERANCE = Degrees.of(3.0);

    public static final VelocityAngleInterpolator SHOOTER_INTERPOLATOR = new VelocityAngleInterpolator(List.of(
      distance(Meters.of(1)).velocity(RotationsPerSecond.of(40)).angle(Rotations.of(0.35)).height(Meters.zero())
    ));

  }

  public static class ClimbConstants {

    public static final int DEVICE_ID_LEFT = 55;
    public static final int DEVICE_ID_RIGHT = 56;

    public static final double RAMP_RATE = 0.25;

  }

}