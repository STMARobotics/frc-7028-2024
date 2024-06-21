package frc.robot.controls;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Interface that defines the available bindings for driver controls. It can be extended to provide
 * a "control scheme"
 */
public interface ControlBindings {

  /**
   * Puts the wheels into an X configuration
   *
   * @return trigger
   */
  Optional<Trigger> wheelsToX();

  /**
   * Supplier for the driver desired X speed
   *
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Distance>>> translationX();

  /**
   * Supplier for the driver desired Y speed
   *
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Distance>>> translationY();

  /**
   * Supplier for the drive desired omega rotation
   *
   * @return velocity supplier
   */
  Supplier<Measure<Velocity<Angle>>> omega();

  /**
   * Intake off the floor and into the turret
   *
   * @return trigger
   */
  Optional<Trigger> intake();

  /**
   * Stop the intake
   *
   * @return trigger
   */
  Optional<Trigger> intakeStop();

  /**
   * Shoot a note that's already in the turret with a fixed velocity, pitch, and yaw
   *
   * @return
   */
  Optional<Trigger> manualShoot();

  /**
   * Automatically shoot a note into the speaker
   *
   * @return trigger
   */
  Optional<Trigger> scoreSpeaker();

  /**
   * Shoot a note into a stockpile by our amp
   *
   * @return
   */
  Optional<Trigger> stockpile();

  /**
   * Tune shooting
   *
   * @return trigger
   */
  Optional<Trigger> tuneShooting();

  /**
   * Score a note that's already in the amper into the amp
   *
   * @return trigger
   */
  Optional<Trigger> scoreAmp();

  /**
   * Run everything in reverse to eject
   *
   * @return trigger
   */
  Optional<Trigger> eject();

  /**
   * Baby bird pickup from human player
   *
   * @return trigger
   */
  Optional<Trigger> babyBird();

  /**
   * Lifts the shooter to max pitch and facing backwards
   *
   * @return trigger
   */
  Optional<Trigger> liftShooter();

  /**
   * Toss a note short for demo purposes. Not intended for comp use.
   *
   * @return trigger
   */
  default Optional<Trigger> demoToss1() {
    return Optional.empty();
  }

  /**
   * Toss a note longer for demo purposes. Not intended for comp use.
   *
   * @return trigger
   */
  default Optional<Trigger> demoToss2() {
    return Optional.empty();
  }

  /**
   * Resets the heading to zero. There is no reason this should be used in comp.
   *
   * @return trigger
   */
  default Optional<Trigger> seedFieldRelative() {
    return Optional.empty();
  }

  /**
   * Shoot a note into a stockpile by our amp
   *
   * @return
   */
  default Optional<Trigger> stockpileMiddle() {
    return Optional.empty();
  }
}
