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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControlBindings implements ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);
  private final MutableMeasure<Velocity<Distance>> translationX = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Distance>> translationY = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Angle>> omega = MutableMeasure.zero(RadiansPerSecond);

  @Override
  public Optional<Trigger> resetPose() {
    return Optional.of(leftJoystick.povDown());
  }
  
  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationX() {
    return () -> translationX.mut_replace(
      MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(leftJoystick.getY()), MetersPerSecond);
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
   return () -> translationY.mut_replace(
      MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(leftJoystick.getX()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -modifyAxis(rightJoystick.getX() * 0.8), RadiansPerSecond);
  }
  
  private static double modifyAxis(double value) {
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  @Override
  public Optional<Trigger> intake() {
    return Optional.of(rightJoystick.button(1));
  }

  @Override
  public Optional<Trigger> intakeReverse() {
    return Optional.of(rightJoystick.button(2));
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> elevatorUp() {
    return Optional.empty();
  }
  
  @Override
  public Optional<Trigger> elevatorDown() {
    return Optional.empty();
  }

}
