package frc.robot.controls;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxControlBindings implements ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);;

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
    return () -> MAX_VELOCITY.times(-modifyAxis(driverController.getLeftY()));
  }

  @Override
  public Optional<Trigger> intakeRollers() {
    return Optional.of(driverController.leftTrigger());
  }
  @Override
  public Optional<Trigger> indexerRun() {
    return Optional.of(driverController.x());
  }
  @Override
  public Optional<Trigger> elevatorVelocity() {
    return Optional.of(driverController.leftBumper());
  }
  
  @Override
  public Optional<Trigger> shootDutyCycle() {
    return Optional.of(driverController.rightTrigger());
  }
  @Override
  public Optional<Trigger> deployIntake() {
    return Optional.of(driverController.rightBumper());
  }
  @Override
  public Optional<Trigger> spit() {
    return Optional.of(driverController.a());
  }

  @Override
  public Supplier<Measure<Velocity<Distance>>> translationY() {
    return () -> MAX_VELOCITY.times(-modifyAxis(driverController.getLeftX()));
  }

  @Override
  public Supplier<Measure<Velocity<Angle>>> omega() {
    return () -> MAX_ANGULAR_VELOCITY.times(-modifyAxis(driverController.getRightX() * 0.8));
  }

  private static double modifyAxis(double value) {
    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
