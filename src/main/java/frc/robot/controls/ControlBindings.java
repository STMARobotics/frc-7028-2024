package frc.robot.controls;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface that defines the available bindings for driver controls. It can be extended to provide a "control scheme"
 */
public interface ControlBindings {

  /**
   * Takes the current orientation of the robot and makes it X forward for field-relative maneuvers
   * @return trigger
   */
  Optional<Trigger> resetPose();

  /**
   * Puts the wheels into an X configuration
   * @return trigger
   */
  Optional<Trigger> wheelsToX();

  /**
   * Supplier for the driver desired X speed
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Distance>>> translationX();

  /**
   * Supplier for the driver desired Y speed
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Distance>>> translationY();

  /**
   * Supplier for the drive desired omega rotation
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Angle>>> omega();

  /**
   * Intake off the floor and into the amper
   * @return trigger
   */
  Optional<Trigger> intakeToAmper();

  /**
   * Intake off the floor and into the turret
   * @return trigger
   */
  Optional<Trigger> intakeToTurret();

  /**
   * Stop the intake
   * @return trigger
   */
  Optional<Trigger> intakeStop();

  /**
   * Shoot a note that's already in the turret with a fixed velocity, pitch, and yaw 
   * @return
   */
  Optional<Trigger> manualShoot();

  /**
   * Automatically shoot a note into the speaker
   * @return trigger
   */
  Optional<Trigger> scoreSpeaker();

  /**
   * Tune speaker shooting
   * @return trigger
   */
  Optional<Trigger> tuneSpeakerShooting();

  /**
   * Exchange a note from turret to amper
   * @return trigger
   */
  Optional<Trigger> exchangeToAmper();

  /**
   * Score a note that's already in the amper into the amp
   * @return trigger
   */
  Optional<Trigger> scoreAmp();
}
