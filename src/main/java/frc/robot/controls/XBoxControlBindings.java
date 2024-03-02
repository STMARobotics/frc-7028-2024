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

/**
 * Control bindings using an XBox controller
 */
public class XBoxControlBindings implements ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);

  // Mutable measure objects to reduce memory pressure by not creating new instances on each iteration
  private final MutableMeasure<Velocity<Distance>> translationX = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Distance>> translationY = MutableMeasure.zero(MetersPerSecond);
  private final MutableMeasure<Velocity<Angle>> omega = MutableMeasure.zero(RadiansPerSecond);

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(driverController.povRight());
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationX() {
    return () -> translationX.mut_replace(
        MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftY()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
    return () -> translationY.mut_replace(
        MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftX()), MetersPerSecond);
  }
  
  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(driverController.getRightX() * 0.8), RadiansPerSecond);
  }
  
  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }
  
  public Optional<Trigger> intakeToTurret() {
    return Optional.of(driverController.rightBumper());
  }

  @Override
  public Optional<Trigger> intakeStop() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.of(driverController.a());
  }

  @Override
  public Optional<Trigger> scoreSpeaker() {
    return Optional.of(driverController.rightTrigger());
  }

  @Override
  public Optional<Trigger> tuneSpeakerShooting() {
    return Optional.of(driverController.start());
  }

  @Override
  public Optional<Trigger> scoreAmp() {
    return Optional.of(driverController.leftTrigger());
    
  }

  @Override
  public Optional<Trigger> exchangeToAmper() {
    return Optional.of(driverController.povUp());
  }

  @Override
  public Optional<Trigger> intakeToAmper() {
    return Optional.of(driverController.povDown());
  }

  @Override
  public Optional<Trigger> eject() {
    return Optional.of(driverController.y());
  }

  @Override
  public Optional<Trigger> babyBird() {
    return Optional.of(driverController.b());
  }

  @Override
  public Optional<Trigger> liftShooter() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> setupShooter() {
    return Optional.of(driverController.x());
  }

}
