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

  default Optional<Trigger> sysIdDriveForwardQuasiTest() {
    return Optional.empty();
  }

    default Optional<Trigger> sysIdDriveReverseQuasiTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdDriveForwardDynamTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdDriveReverseDynamTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdSteerForwardQuasiTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdSteerReverseQuasiTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdSteerForwardDynamTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdSteerReverseDynamTest() {
    return Optional.empty();
  }

  default Optional<Trigger> sysIdDriveSlipTest() {
    return Optional.empty();
  }
}
