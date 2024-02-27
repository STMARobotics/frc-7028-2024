package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  // Array of photon pose estimators for photon cameras
  private final PhotonPoseEstimator[] photonPoseEstimators;
  // Array of subscribers for camera subtables
  private final RawSubscriber[] rawBytesSubscribers;
  // Array of subscriber wait handles for camera subtables, used to lookup camera index from wait handle
  private final int[] waitHandles;
  
  // Consumer of pose estimates
  private final BiConsumer<Pose2d, Double> poseConsumer;

  private final Supplier<Pose2d> poseSupplier;

  @SuppressWarnings("unchecked")
  private final StructArrayPublisher<AprilTag>[] aprilTagPublishers = new StructArrayPublisher[2];
   

  private final Packet packet = new Packet(1);

  public PhotonRunnable(String[] cameraNames, Transform3d[] robotToCameras, BiConsumer<Pose2d, Double> poseConsumer,
        Supplier<Pose2d> poseSupplier) {
    this.poseConsumer = poseConsumer;
    this.poseSupplier = poseSupplier;
    
    // NT publishers to send data to AdvantageScope
    for (int i = 0; i < cameraNames.length; i++) {
      aprilTagPublishers[i] = NetworkTableInstance.getDefault()
          .getStructArrayTopic("AprilTags-" + cameraNames[i], new AprilTagStruct()).publish();
    }

    rawBytesSubscribers = new RawSubscriber[cameraNames.length];
    photonPoseEstimators = new PhotonPoseEstimator[cameraNames.length];
    waitHandles = new int[cameraNames.length];

    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Origin will always be blue
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    for (int i = 0; i < cameraNames.length; i++) {
      var cameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraNames[i]);
      rawBytesSubscribers[i] = cameraTable.getRawTopic("rawBytes")
          .subscribe(
              "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
      waitHandles[i] = rawBytesSubscribers[i].getHandle();
      photonPoseEstimators[i] = new PhotonPoseEstimator(
          layout, MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera(cameraNames[i]), robotToCameras[i]);
    }
  }

  @Override
  public void run() {
    var emptyAprilTagArray = new AprilTag[0];
    while (!Thread.interrupted()) {
      // Block the thread until new data comes in from PhotonVision
      int[] signaledHandles = null;
      try {
        signaledHandles = WPIUtilJNI.waitForObjects(waitHandles);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }

      var currentRobotPose = poseSupplier.get();
      for (int i = 0; i < signaledHandles.length; i++) {
        int cameraIndex = getCameraIndex(signaledHandles[i]);
        var aprilTagPublisher = aprilTagPublishers[cameraIndex];
        var photonPoseEstimator = photonPoseEstimators[cameraIndex];

        // Get AprilTag data
        var photonResults = getLatestResult(cameraIndex);
        if (photonResults.hasTargets() && (photonResults.targets.size() > 1
            || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
          
          // Send the AprilTag(s) to NT for AdvantageScope
          aprilTagPublisher.accept(photonResults.targets.stream().map(target ->
              getTargetPose(target, currentRobotPose, photonPoseEstimator.getRobotToCameraTransform())
          ).toArray(AprilTag[]::new));
          
          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH.in(Meters)
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH.in(Meters)) {
                  poseConsumer.accept(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          });
        } else {
          // No tags, send empty array to NT
          aprilTagPublisher.accept(emptyAprilTagArray);
        }
      }
    }
    Arrays.stream(rawBytesSubscribers).forEach(RawSubscriber::close);
    Arrays.stream(aprilTagPublishers).forEach(StructArrayPublisher::close);
  }

  /**
   * Transform a target from PhotonVision to a pose on the field
   * @param target target data from PhotonVision
   * @param robotPose current pose of the robot
   * @param robotToCamera transform from robot to the camera that saw the target
   * @return an AprilTag with an ID and pose
   */
  private static AprilTag getTargetPose(PhotonTrackedTarget target, Pose2d robotPose, Transform3d robotToCamera) {
    var targetPose = new Pose3d(robotPose)
        .transformBy(robotToCamera)
        .transformBy(target.getBestCameraToTarget());
    return new AprilTag(target.getFiducialId(), targetPose);
  }

  /**
   * Find the camera index for a table wait handle
   * @param signaledHandle handle
   * @return index, or -1 if not found
   */
  private int getCameraIndex(int signaledHandle) {
    for (int i = 0; i < waitHandles.length; i++) {
      if (waitHandles[i] == signaledHandle) {
        return i;
      }
    }
    return -1;
  }

  public PhotonPipelineResult getLatestResult(int cameraIndex) {
    packet.clear();
    var result = new PhotonPipelineResult();
    packet.setData(rawBytesSubscribers[cameraIndex].get(new byte[] {}));
    if (packet.getSize() < 1) {
      return result;
    }
    result = PhotonPipelineResult.serde.unpack(packet);
    result.setTimestampSeconds((rawBytesSubscribers[cameraIndex].getLastChange() / 1e6) - result.getLatencyMillis() / 1e3);
    return result;
  }

}
