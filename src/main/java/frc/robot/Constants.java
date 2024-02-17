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

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class Constants {

  // Change to a different type in the future
  public static final Boolean team = true;

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
    public static final int DEVICE_ID_SHOOTER_LEADER = 1;
    public static final int DEVICE_ID_SHOOTER_FOLLOWER = 2;
    public static final int DEVICE_ID_SHOOTER_ALTITUDE_CONTROL = 3;
    public static final int DEVICE_ID_SHOOTER_MOTOR_INTAKE = 4;
    public static final int SHOOTER_VELOCITY_TOLERANCE = 5;

    public static final SlotConfigs SHOOTER_SLOT_CONFIGS = new SlotConfigs()
        .withKP(0.59)
        .withKI(0)
        .withKD(0.001)
        .withKS(0);
    public static final SlotConfigs ALTITUDE_SLOT_CONFIGS = new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0);
    public static final SlotConfigs SHOOTER_INTAKE_SLOT_CONFIGS = new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0);
  }

  public static class TurretConstants {
    public static final int DEVICE_ID_TURRET = 6;

    public static final class Circumfrence {
      public static final Measure<Distance> inner = Inches.of(1);
      public static final Measure<Distance> outer = Inches.of(1);
      public static final Measure<Distance> center = Inches.of(1);
    }

    // To be determined!!
    public static final Translation2d speakerPosition1 = new Translation2d(0, 0);
    public static final Translation2d speakerPosition2 = new Translation2d(0, 0); 

    public static final Double oneRotorRotation = 1d;
    public static final Double oneRotationForSensor = 1d;
    // NOT the final expression
    public static final Double rotorToSensorRatio = oneRotorRotation / oneRotationForSensor;

    public static final int TURRET_LOWER_ANGLE_RANGE = 0;
    public static final int TURRET_UPPER_ANGLE_RANGE = 270;

    public static final Double TURRET_ANGLE_THRESHOLD = 5d;
    public static final Double TURRET_DRIVETRAIN_ANGLE_THRESHOLD = 5d;

    // To be determined!! 
    public static final Measure<Distance> TURRET_MAXIMUM_READYSHOOT_DISTANCE = Meters.of(1);

    public static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration()
      .withFeedback(
        new FeedbackConfigs()
          .withRotorToSensorRatio(rotorToSensorRatio)
      );

  }

  public static class IntakeConstants {
    public static final int DEVICE_ID_INTAKE = 7;

    public static final int DEVICE_ID_ROLLERS_MOTOR = 42;

    public static final double MagnetOffsetValue = 0.4;

    public static final SlotConfigs INTAKE_SLOT_CONFIGS = new SlotConfigs()
        .withKP(0.1)
        .withKI(0.1)
        .withKD(0)
        .withKS(0.59);

  }

  public static class ElevatorConstants {

    public static final int ELEVATOR_POSITON_ID = 60;
    public static final double ELEVATOR_PARK_HEIGHT = .06;

    // Elevator travel distance, in meters
    public static final double ELEVATOR_HEIGHT = 1.002;
    
    // Motor's encoder limits, in encoder ticks
    public static final double MOTOR_BOTTOM = 0;
    public static final double MOTOR_TOP = 56530;
    
    // Mutiply by sensor position to get meters
    public static final double MOTOR_ENCODER_POSITION_COEFFICIENT = ELEVATOR_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM);
    
    public static final int TOP_LIMIT_SWITCH_CHANNEL = 9;
    public static final int BOTTOM_LIMIT_SWITCH_CHANNEL = 8;

    public static final SlotConfigs ELEVATOR_SLOT_CONFIGS = new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0);

  }

  public class ClimbConstants {

    public static final int LEFT_CLIMB_ARM_ID = 55;
    public static final int RIGHT_CLIMB_ARM_ID = 56;
    public static final double CHAIN_HEIGHT = 0.75;

    public static final int CLIMB_HEIGHT = 1;

    public static final double MOTOR_TOP = 56530;
    public static final double MOTOR_BOTTOM = 0;

    public static final double MOTOR_ENCODER_POSITION_COEFFICIENT = CLIMB_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM);

    public static final SlotConfigs CLIMB_SLOT_CONFIGS = new SlotConfigs()
      .withKP(0)
      .withKI(0)
      .withKD(0)
      .withKS(0);

  }


  public static class TrapAmpShooterConstants {
    public static final int DEVICE_ID_TRAP_AMP_SHOOTER = 10;
  }
  
}
