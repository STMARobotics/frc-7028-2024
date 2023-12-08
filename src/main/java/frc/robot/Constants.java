package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
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
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    
  }
}
