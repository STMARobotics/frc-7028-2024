package frc.robot.subsystems;

import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();
  private Pose3d currentPose = new Pose3d();

  private final StructPublisher<Pose3d> questPublisher = NetworkTableInstance.getDefault()
      .getTable("Quest")
      .getStructTopic("Quest Robot Pose", Pose3d.struct)
      .publish();

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      // Get the most recent Quest pose
      var poseFrame = poseFrames[poseFrames.length - 1];
      if (poseFrame.isTracking()) {
        var questPose = poseFrame.questPose3d();
        // Transform by the mount pose to get your robot pose
        currentPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

        // Publish the current pose to NetworkTables
        questPublisher.set(currentPose);
      }
    }
  }

  public Pose3d getLatestPose() {
    return currentPose;
  }

  public void resetPose(Pose3d newRobotPose) {
    // Transform the desired robot pose to the corresponding Quest pose
    Pose3d newQuestPose = newRobotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(newQuestPose);
  }
}
