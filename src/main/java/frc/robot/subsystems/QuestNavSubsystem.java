package frc.robot.subsystems;

import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();
  private Pose3d lastPose = new Pose3d();

  @Override
  public void periodic() {
    questNav.commandPeriodic();
  }

  public Pose3d getLatestPose() {
    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      // Get the most recent Quest pose
      var poseFrame = poseFrames[poseFrames.length - 1];
      if (poseFrame.isTracking()) {
        var questPose = poseFrame.questPose3d();
        // Transform by the mount pose to get your robot pose
        return lastPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
      }
    }
    // Don't do this on a real robot! No data needs to be handled instead of returning stale data!
    DriverStation.reportWarning("No QuestNav data!", false);
    return lastPose;
  }

  public void resetPose(Pose3d newRobotPose) {
    // Transform the desired robot pose to the corresponding Quest pose
    Pose3d newQuestPose = newRobotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(newQuestPose);
  }
}
