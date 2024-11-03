package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/** Control bindings using an XBox controller */
public class XBoxControlBindings implements ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);

  // Mutable measure objects to reduce memory pressure by not creating new instances on each
  // iteration
  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.of(driverController.povRight());
  }

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX
        .mut_replace(MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftY()), MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY
        .mut_replace(MAX_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftX()), MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(driverController.getRightX() * 0.8),
          RadiansPerSecond);
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  public Optional<Trigger> intake() {
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
  public Optional<Trigger> stockpile() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> tuneShooting() {
    return Optional.of(driverController.start());
  }

  @Override
  public Optional<Trigger> scoreAmp() {
    return Optional.of(driverController.leftTrigger());
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
  public Optional<Trigger> babyBomber() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> liftShooter() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> seedFieldRelative() {
    return Optional.of(driverController.back());
  }
}
