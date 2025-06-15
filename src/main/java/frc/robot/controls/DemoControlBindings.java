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

/**
 * Control bindings for demos with joysticks
 */
public class DemoControlBindings implements ControlBindings {

  private static final double DEMO_SPEED_FACTOR = 0.25;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Optional<Trigger> wheelsToX() {
    return Optional.empty();
  }

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX
        .mut_replace(MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(driverController.getLeftY()), MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY
        .mut_replace(MAX_VELOCITY.in(MetersPerSecond) * -modifyAxis(driverController.getLeftX()), MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -modifyAxis(driverController.getRightX()),
          RadiansPerSecond);
  }

  /**
   * Square the input and slow it down for demo
   * 
   * @param value axis value
   * @return modified value
   */
  private static double modifyAxis(double value) {
    return Math.copySign(value * value, value) * DEMO_SPEED_FACTOR;
  }

  @Override
  public Optional<Trigger> intake() {
    return Optional.of(driverController.a());
  }

  @Override
  public Optional<Trigger> intakeStop() {
    return Optional.of(driverController.povUp());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> scoreSpeaker() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> stockpile() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> tuneShooting() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> scoreAmp() {
    return Optional.empty();
  }

  @Override
  public Optional<Trigger> eject() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> babyBird() {
    return Optional.of(driverController.leftTrigger());
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
  public Optional<Trigger> demoToss1() {
    return Optional.of(driverController.rightTrigger());
  }

  @Override
  public Optional<Trigger> demoToss2() {
    return Optional.of(driverController.povLeft());
  }

  @Override
  public Optional<Trigger> seedFieldRelative() {
    return Optional.of(driverController.start());
  }

}