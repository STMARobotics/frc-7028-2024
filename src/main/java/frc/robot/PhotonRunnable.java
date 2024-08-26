package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;
import static frc.robot.Constants.VisionConstants.SINGLE_TAG_DISTANCE_THRESHOLD;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;

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
  private final AddVisionMeasurement poseConsumer;

  private final Supplier<Pose2d> poseSupplier;

  public boolean isCameraConnected;

  @SuppressWarnings("unchecked")
  private final StructArrayPublisher<AprilTag>[] aprilTagPublishers = new StructArrayPublisher[2];
   

  private final Packet packet = new Packet(1);

  public PhotonRunnable(String[] cameraNames, Transform3d[] robotToCameras, AddVisionMeasurement poseConsumer,
        Supplier<Pose2d> poseSupplier) {
    this.poseConsumer = poseConsumer;
    this.poseSupplier = poseSupplier;
    
    // Forward PhotonVision ports for when teathered with USB
    // https://docs.photonvision.org/en/latest/docs/installation/networking.html#port-forwarding
    PortForwarder.add(5800, "10.70.28.11", 5800);
    PortForwarder.add(1181, "10.70.28.11", 1181);
    PortForwarder.add(1182, "10.70.28.11", 1182);
    PortForwarder.add(1183, "10.70.28.11", 1183);
    PortForwarder.add(1184, "10.70.28.11", 1184);
    PortForwarder.add(1185, "10.70.28.11", 1185);
    PortForwarder.add(1186, "10.70.28.11", 1186);
    PortForwarder.add(1187, "10.70.28.11", 1187);
    PortForwarder.add(1188, "10.70.28.11", 1188);

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
    final PhotonCamera leftCamera = new PhotonCamera("Left");
    final PhotonCamera rightCamera = new PhotonCamera("Right");
    if (leftCamera.isConnected() && rightCamera.isConnected()) {
      isCameraConnected = true;
      leftCamera.close();
      rightCamera.close();
    }
  }

  public boolean isCameraConnected() {
    return isCameraConnected;
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

      if(RobotState.isAutonomous()) {
        // Short circuit when in auto
        continue;
      }

      var currentRobotPose = poseSupplier.get();
      for (int i = 0; i < signaledHandles.length; i++) {
        int cameraIndex = getCameraIndex(signaledHandles[i]);
        var aprilTagPublisher = aprilTagPublishers[cameraIndex];
        var photonPoseEstimator = photonPoseEstimators[cameraIndex];

        // Get AprilTag data
        var photonResults = getLatestResult(cameraIndex);
        if (photonResults.hasTargets() && (photonResults.targets.size() > 1
            || (photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD))) {
          
          // Send the AprilTag(s) to NT for AdvantageScope
          aprilTagPublisher.accept(photonResults.targets.stream().map(target ->
              getTargetPose(target, currentRobotPose, photonPoseEstimator.getRobotToCameraTransform())
          ).toArray(AprilTag[]::new));
          
          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH.in(Meters)
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH.in(Meters)) {

              var stdDevs = getEstimationStdDevs(
                  estimatedPose.toPose2d(), photonResults.getTargets(), photonPoseEstimator.getFieldTags());
              poseConsumer.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, stdDevs);
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

  /**
   * Scales the standard deviation based on the number of targets and their distance.
   *
   * @param estimatedPose estimated pose
   * @param targets targets from PhotonVision
   * @param fieldLayout tag poses
   */
  private static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets, AprilTagFieldLayout fieldLayout) {

    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
    }

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
    return estStdDevs;
  }

  @FunctionalInterface
  public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

}
