package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class Constants {

  public static final class DrivetrainConstants {
    public static final Measure<Distance> TRACKWIDTH = Inches.of(18.75);
    public static final Measure<Distance> WHEELBASE = Inches.of(18.75);

    // Theoretical free speed of L2 Falcon 500 with FOC
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = FeetPerSecond.of(15.7);
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(PI * 2);
  
    public static final double STEER_kP = 30.0;
    public static final double STEER_kI = 0.0;
    public static final double STEER_kD = 0.0;
    public static final double STEER_kV = 0.7854;

    public static final double DRIVE_kP = 13.0;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;
  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 6.0;
    public static final double Y_RATE_LIMIT = 6.0;
    public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

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

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(-0.06, 0.2, -0.2127),
        new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0)));

    /** Physical location of the shooter camera on the robot, relative to the center of the robot. */
    public static final Transform3d LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.083, 0.254, -0.537),
        new Rotation3d(0.0, degreesToRadians(-9.0), degreesToRadians(-1.0)));

    public static final String LIMELIGHT_NAME = "limelight";
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    
    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    public static Pose2d flipAlliance(Pose2d poseToFlip) {
      return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
    }

  }
}
