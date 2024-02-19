package frc.robot.controls;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBindings {
  Optional<Trigger> resetPose();
  Optional<Trigger> wheelsToX();
  Supplier<Measure<Velocity<Distance>>> translationX();
  Supplier<Measure<Velocity<Distance>>> translationY();
  Supplier<Measure<Velocity<Angle>>> omega();
  Optional<Trigger> intake();
  Optional<Trigger> intakeStop();
  Optional<Trigger> manualShoot();
  Optional<Trigger> autoShoot();
  /**
   * Exchange from turret to amper
   * @return
   */
  Optional<Trigger> loadAmper();

  /**
   * Intkake off the floor, straight to amper
   */
  Optional<Trigger> intakeToAmper();
  Optional<Trigger> scoreAmp();
}
