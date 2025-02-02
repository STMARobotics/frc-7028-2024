package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConsumer;

/**
 * Command to use PhotonVision to process pose estimates and pass them to a pose estimator
 */
public class PhotonVisionCommand extends Command {
  private final Vision vision;
  private final VisionConsumer visionConsumer;

  /**
   * Constructs a PhotonVisionCommand
   * 
   * @param consumer consumer to receive vision estimates
   */
  public PhotonVisionCommand(VisionConsumer consumer) {
    this.vision = new Vision();
    this.visionConsumer = consumer;
  }

  @Override
  public void execute() {
    // Correct pose estimate with vision measurements
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      // Change our trust in the measurement based on the tags we can see
      var estStdDevs = vision.getEstimationStdDevs();

      visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}