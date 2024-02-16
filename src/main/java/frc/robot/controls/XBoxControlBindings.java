package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxControlBindings implements ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final MutableMeasure<Velocity<Distance>> translationX = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Distance>> translationY = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Angle>> omega = MutableMeasure.zero(RadiansPerSecond);

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(driverController.back());
  }

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(driverController.povRight());
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationX() {
    return () -> translationX.mut_replace(
        MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(driverController.getLeftY()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
    return () -> translationY.mut_replace(
        MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(driverController.getLeftX()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -modifyAxis(driverController.getRightX() * 0.8), RadiansPerSecond);
  }
  
  private static double modifyAxis(double value) {
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  public Optional<Trigger> intake() {
    return Optional.of(driverController.rightBumper());
  }

  @Override
  public Optional<Trigger> intakeStop() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> intakeReverse() {
    return Optional.of(driverController.x());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.of(driverController.a());
  }

  @Override
  public Optional<Trigger> autoShoot() {
    return Optional.of(driverController.rightTrigger());
  }

  @Override
  public Optional<Trigger> scoreAmp() {
    return Optional.of(driverController.leftTrigger());
  }

  @Override
  public Optional<Trigger> loadAmper() {
    return Optional.of(driverController.povUp());
  }

}
