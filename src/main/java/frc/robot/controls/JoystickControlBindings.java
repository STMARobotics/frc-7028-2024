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

/**
 * Control bindings for driving with joysticks
 */
public class JoystickControlBindings implements ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);
  private final MutableMeasure<Velocity<Distance>> translationX = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Distance>> translationY = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Angle>> omega = MutableMeasure.zero(RadiansPerSecond);

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(leftJoystick.button(3));
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationX() {
    return () -> translationX.mut_replace(
      MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getY()), MetersPerSecond);
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
   return () -> translationY.mut_replace(
      MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getX()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(rightJoystick.getX()), RadiansPerSecond);
  }
  
  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  @Override
  public Optional<Trigger> intakeToTurret() {
    return Optional.of(leftJoystick.trigger());
  }

  @Override
  public Optional<Trigger> intakeStop() {
    return Optional.of(leftJoystick.povUp());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.of(rightJoystick.povDown());
  }

  @Override
  public Optional<Trigger> scoreSpeaker() {
    return Optional.of(rightJoystick.trigger());
  }

  @Override
  public Optional<Trigger> tuneSpeakerShooting() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> scoreAmp() {
    return Optional.of(rightJoystick.povUp());
  }

  @Override
  public Optional<Trigger> exchangeToAmper() {
    return Optional.of(rightJoystick.povLeft());
  }

  @Override
  public Optional<Trigger> intakeToAmper() {
    return Optional.of(leftJoystick.povRight());
  }

  @Override
  public Optional<Trigger> eject() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public Optional<Trigger> babyBird() {
    return Optional.of(leftJoystick.button(2));
  }

  @Override
  public Optional<Trigger> liftShooter() {
    return Optional.of(rightJoystick.button(2));
  }

  @Override
  public Optional<Trigger> setupShooter() {
    return Optional.of(rightJoystick.button(4));
  }

}
