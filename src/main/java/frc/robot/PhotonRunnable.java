package frc.robot;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.BiConsumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotState;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final BiConsumer<Pose2d, Double> poseConsumer;
  private final RawSubscriber rawBytesSubscriber;
  private final Packet packet = new Packet(1);

  public PhotonRunnable(String cameraName, BiConsumer<Pose2d, Double> poseConsumer) {
    this.poseConsumer = poseConsumer;
    var cameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraName);
    rawBytesSubscriber = cameraTable.getRawTopic("rawBytes")
        .subscribe(
            "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));

    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Origin will always be blue
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    photonPoseEstimator = new PhotonPoseEstimator(
        layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera(cameraName), APRILTAG_CAMERA_TO_ROBOT.inverse());
  }

  @Override
  public void run() {
    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from PhotonVision
      try {
        WPIUtilJNI.waitForObjects(new int[] {rawBytesSubscriber.getHandle()});
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

      if (!RobotState.isAutonomous()) {
        // Get AprilTag data
        var photonResults = getLatestResult();
        if (photonResults.hasTargets() 
            && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                  poseConsumer.accept(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          });
        }
      }
    }
    rawBytesSubscriber.close();
  }

  public PhotonPipelineResult getLatestResult() {
    packet.clear();
    var result = new PhotonPipelineResult();
    packet.setData(rawBytesSubscriber.get(new byte[] {}));
    if (packet.getSize() < 1) {
      return result;
    }
    result = PhotonPipelineResult.serde.unpack(packet);
    result.setTimestampSeconds((rawBytesSubscriber.getLastChange() / 1e6) - result.getLatencyMillis() / 1e3);
    return result;
  }

}
