package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static java.lang.Math.PI;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";
  
  public static final class DrivetrainConstants {
    public static final Measure<Distance> TRACKWIDTH = Inches.of(18.75);
    public static final Measure<Distance> WHEELBASE = Inches.of(18.75);

    // Theoretical free speed of L2 Falcon 500 with FOC
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = FeetPerSecond.of(15.7);
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY
        .in(MetersPerSecond) /
        Math.hypot(TRACKWIDTH.in(Meters) / 2.0, WHEELBASE.in(Meters) / 2.0));
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(PI * 4);
  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final Measure<Velocity<Velocity<Distance>>> TRANSLATION_RATE_LIMIT = MetersPerSecondPerSecond.of(8.0);
    public static final Measure<Velocity<Velocity<Angle>>> ROTATION_RATE_LIMIT = RadiansPerSecond.per(Second)
        .of(8.0 * PI);

  }

  public static class AutoDriveConstants {

    public static final double THETA_kP = 2.6;
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
    public static final String[] APRILTAG_CAMERA_NAMES = {"Camera-1", "Camera-2"};

    /**
     * Physical location of the apriltag cameras on the robot, relative to the center of the robot.
     * The values here math APRILTAG_CAMERA_NAMES for the camera's name.
     */
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
        new Transform3d(
            new Translation3d(0.06, -0.2, 0.2127),
            new Rotation3d(0.0, degreesToRadians(-15.0), degreesToRadians(3.0))),
        new Transform3d(
            new Translation3d(0.06, -0.2, 0.2127),
            new Rotation3d(0.0, degreesToRadians(-15.0), degreesToRadians(3.0))),
      };

    public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
    public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when multiple tags are
     * in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

  }

  public static class IndexerConstants {
    public static final int DEVICE_ID_LEFT = 1;
    public static final int DEVICE_ID_RIGHT = 2;

    public static final int PORT_ID_FULL_SENSOR = 0;
    public static final int PORT_ID_INTAKE_SENSOR = 1;
    public static final int PORT_ID_SPACER_SENSOR = 2;

    /** Color of note */
    public static final Color COLOR_NOTE = new Color(.487, .393, .12);

    /** Color of no note */
    public static final Color COLOR_NONE = new Color(0.253, 0.49, 0.255);

    // Proximity thresholds for when to trip each sensor
    public static final int THRESHOLD_INTAKE = 240;
    public static final int THRESHOLD_SPACE = 400;
    public static final int THRESHOLD_FULL = 225;

    public static final double kP = 0.000006d;
    public static final double kI = 0.0d;
    public static final double kD = 0.0d;

    public static final double LEFT_kS = 0.4387;
    public static final double LEFT_kV = 0.00115;
    public static final double LEFT_kA = 0.00013792;

    public static final double RIGHT_kS = 0.4387;
    public static final double RIGHT_kV = 0.0013;
    public static final double RIGHT_kA = 0.00013792;

    public static final Measure<Velocity<Angle>> RUN_SPEED = RotationsPerSecond.of(30);
    public static final Measure<Velocity<Angle>> SHOOT_SPEED = RotationsPerSecond.of(30);
  }

  public static class ShooterConstants {
    public static final int DEVICE_ID_TOP = 50;
    public static final int DEVICE_ID_BOTTOM = 51;
    public static final int DEVICE_ID_AIM = 52;
    public static final int ACTUATOR_CANCORDER = 3;

    public static final SlotConfigs SHOOTER_VELOCITY_SLOT_CONFIG_TOP = new SlotConfigs()
        .withKP(9.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.040599);

    public static final SlotConfigs SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM = new SlotConfigs()
        .withKP(15.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.096656);
  
    public static final SlotConfigs SHOOTER_POSITION_SLOT_CONFIG_TOP = new SlotConfigs()
        .withKP(35.59)
        .withKI(0.0)
        .withKD(1.2791);

    public static final SlotConfigs SHOOTER_POSITION_SLOT_CONFIG_BOTTOM = new SlotConfigs()
        .withKP(36.957)
        .withKI(0.0)
        .withKD(1.4008);
    
    public static final double SHOOTER_SENSOR_TO_MECHANISM_RATIO = 1.0;
  
    public static final double AIM_kP = 0.015;
    public static final double AIM_kI = 0d;
    public static final double AIM_kD = 0d;

    public static final Measure<Angle> AIM_OFFSET = Rotations.of(0.9503065);
    public static final float AIM_FORWARD_LIMIT = 0.32f;
    public static final float AIM_REVERSE_LIMIT = 0.12f;
    public static final Measure<Voltage> AIM_GRAVITY_FF = Volts.of(3.5);
  }

  public static class IntakeConstants {
    public static final int DEVICE_ID_DEPLOY = 40;
    public static final int DEVICE_ID_DEPLOY_CANIVORE = 41;
    public static final int DEVICE_ID_ROLLER = 42;

    public static final Measure<Angle> DEPLOY_CANCODER_OFFSET = Rotations.of(0.417725);
    public static final double DEPLOY_ROTOR_TO_SENSOR_RATIO = 351.1133117;
    public static final SlotConfigs DEPLOY_SLOT_CONFIGS = new SlotConfigs()
        .withKP(75)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.6)
        .withKV(8.0)
        .withKA(0.0)
        .withKG(0.45)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    public static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(8)
        .withMotionMagicCruiseVelocity(2);
    
    public static final Measure<Angle> DEPLOY_POSITION_DEPLOYED = Rotations.of(-0.126);
    public static final Measure<Angle> DEPLOY_POSITION_RETRACTED = Rotations.of(0.295);
    public static final Measure<Angle> DEPLOY_TOLERANCE = Rotations.of(.01);

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs()
        .withKP(9.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.243);
    public static final double ROLLER_SENSOR_TO_MECHANISM_RATIO = 1.0;
    public static final Measure<Velocity<Angle>> ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(40.0);
    public static final Measure<Velocity<Angle>> ROLLER_REVERSE_VELOCITY = RotationsPerSecond.of(-10.0);
  }

  public static class ElevatorConstants {
    public static int DEVICE_ID_MOTOR_0 = 60;
    public static int DEVICE_ID_MOTOR_1 = 61;
    public static int DEVICE_PORT_TOP_LIMIT = 5;
    public static int DEVICE_PORT_BOTTOM_LIMIT = 6;

    public static Measure<Per<Distance, Angle>> METERS_PER_REVOLUTION = Meters.per(Rotations).of(0.001);

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
    
    public static final Measure<Distance> TOP_LIMIT = Meters.of(0.5);
    public static final Measure<Distance> BOTTOM_LIMIT = Meters.zero();
  }
}