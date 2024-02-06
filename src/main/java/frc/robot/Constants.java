package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static java.lang.Math.PI;

import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";

  public static final class DrivetrainConstants {
    public static final Measure<Distance> TRACKWIDTH = Inches.of(18.75);
    public static final Measure<Distance> WHEELBASE = Inches.of(18.75);

    // Theoretical free speed of L2 Falcon 500 with FOC
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = FeetPerSecond.of(15.7);
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
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

    public static final String APRILTAG_CAMERA_NAME = "OV9281";

    /**
     * Physical location of the apriltag camera on the robot, relative to the center
     * of the robot.
     */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(-0.06, 0.2, -0.2127),
        new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0)));

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class ShooterConstants {
    public static final int DEVICE_ID_SHOOTER_RIGHT = 51;
    public static final int DEVICE_ID_SHOOTER_LEFT = 50;
    public static final int DEVICE_ID_ACTUATOR_MOTOR = 52;

    public static final int SHOOTER_VELOCITY_OFFSET = 0;
    public static final int WRIST_POSITION_OFFSET = 0;
    public static final int SHOOTER_MOTOR_RATIO = 1;
    
    public static final SlotConfigs SHOOTER_SLOT_CONFIGS = new SlotConfigs()
    .withKP(0)
    .withKI(0)
    .withKD(0)
    .withKS(0);
  }

  public static class ClimbConstants {
    public static DeadbandFilter CLIMB_DEADBAND_FILTER = new DeadbandFilter(-.25, .5);

    public static final int DEVICE_ID_FIRST_STAGE_CLIMB = 12;

    public static final double SOFT_LIMIT_FIRST_STAGE_FWD = 430000;
    public static final double SOFT_LIMIT_FIRST_STAGE_REV = 0;
  }

  public static class IntakeConstants {
    public static final int DEVICE_ID_DEPLOY_MOTOR = 40;
    public static final int DEVICE_ID_DEPLOY_CANCODER = 41;
    public static final int DEVICE_ID_ROLLERS_MOTOR = 42;
  }

  public static class ElevatorConstants {
    public static final int DEVICE_ID_MOTOR_1 = 60;
    public static final int DEVICE_ID_MOTOR_2 = 61;
    public static final int TOP_LIMIT = 5;
    public static final int BOTTOM_LIMIT = 6;
    public static final int ELEVATOR_PARK_HEIGHT = 0; // We don't what it will be yet 
  }

  public static class IndexerConstants {
    public static final int PORT_ID_INTAKE_SENSOR = 3;
    public static final int PORT_ID_SHOOTER_SENSOR = 2;

    public static final int DEVICE_ID_INDEXER_LEFT_MOTOR = 1;
    public static final int DEVICE_ID_INDEXER_RIGHT_MOTOR = 2;

    // Proximity thresholds for when to trip each sensor
    public static final int THRESHOLD_INTAKE = 240;
    public static final int THRESHOLD_SHOOTER = 225;

    public static final double BELT_kP = 0d;
    public static final double BELT_RUN_SPEED = 8000;
    public static final double BELT_SHOOT_SPEED = 8000;

    /** Color of note */
    public static final Color COLOR_NOTE = new Color(.487, .393, .12);

    /** Color of no note */
    public static final Color COLOR_NONE = new Color(0.253, 0.49, 0.255);
  }
}