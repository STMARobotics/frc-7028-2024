package frc.robot.controls;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBindings {
  Optional<Trigger> elevatorVelocity();
  Optional<Trigger> resetPose();
  Optional<Trigger> wheelsToX();
  Optional<Trigger> shootDutyCycle();
  Optional<Trigger> spit();
  Optional<Trigger> intakeRollers();
  Optional<Trigger> deployIntake();
  Optional<Trigger> retractIntake();
  Optional<Trigger> indexerRun();
  Optional<Trigger> actuatorUp();
  Optional<Trigger> actuatorDown();
  Supplier<Measure<Velocity<Distance>>> translationX();
  Supplier<Measure<Velocity<Distance>>> translationY();
  Supplier<Measure<Velocity<Angle>>> omega();
}
