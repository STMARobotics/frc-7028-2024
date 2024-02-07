// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ElevatorConstants.BOTTOM_LIMIT;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_MOTOR_0;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_MOTOR_1;
import static frc.robot.Constants.ElevatorConstants.DEVICE_PORT_BOTTOM_LIMIT;
import static frc.robot.Constants.ElevatorConstants.DEVICE_PORT_TOP_LIMIT;
import static frc.robot.Constants.ElevatorConstants.METERS_PER_REVOLUTION;
import static frc.robot.Constants.ElevatorConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.SLOT_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.TOP_LIMIT;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorLeader = new TalonFX(DEVICE_ID_MOTOR_0, CANIVORE_BUS_NAME);
  private final TalonFX elevatorFollower = new TalonFX(DEVICE_ID_MOTOR_1, CANIVORE_BUS_NAME);

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(DEVICE_PORT_TOP_LIMIT);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(DEVICE_PORT_BOTTOM_LIMIT);

  private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);

  // SysId routines  
  private final SysIdRoutine elevatorRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(5.0), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        elevatorLeader.setControl(voltageControl.withOutput(volts.in(Volts)));
        elevatorFollower.setControl(voltageControl.withOutput(volts.in(Volts)));
      }, null, this));
  
  public ElevatorSubsystem() {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    motorConfig.MotionMagic = MOTION_MAGIC_CONFIGS;
    motorConfig.MotorOutput.NeutralMode = Brake;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TOP_LIMIT.in(Meters) / METERS_PER_REVOLUTION.in(Meters.per(Rotations));
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        BOTTOM_LIMIT.in(Meters) / METERS_PER_REVOLUTION.in(Meters.per(Rotations));

    elevatorLeader.getConfigurator().apply(motorConfig);
    elevatorFollower.getConfigurator().apply(motorConfig);

    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));
  }

  /**
   * Indicates if the top limit switch is tripped
   * @return true if the limit switch is tripped, otherwise false
   */
  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  /**
   * Indicates if the bottom limit switch is tripped
   * @return true if the bottom limit switch is tripped, otherwise false
   */
  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  /**
   * Returns a new command to move the elevator to a given position
   * @param position position to move the elevator to
   * @return new command
   */
  public Command moveToPositionCommand(Measure<Distance> position) {
    return run(() -> this.moveToPosition(position)).finallyDo(this::stop);
  }

  /**
   * Returns a new command to manually move the elevator up at a fixed speed. Probably only for use when testing.
   * @return new command
   */
  public Command manualUpCommand() {
    return run(() -> move(Volts.of(5.0))).finallyDo(this::stop);
  }

  /**
   * Returns a new command to manually move the elevator down at a fixed speed. Probably only for use when testing.
   * @return new command
   */
  public Command manualDownCommand() {
    return run(() -> move(Volts.of(-2.0))).finallyDo(this::stop);
  }

  public Command sysIdDynamicCommand(Direction direction) {
    return elevatorRoutine.dynamic(direction).withName("SysId elevator dynamic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdQuasistaticCommand(Direction direction) {
    return elevatorRoutine.quasistatic(direction).withName("SysId elevator quasistatic " + direction)
        .finallyDo(this::stop);
  }

  private void moveToPosition(Measure<Distance> position) {
    elevatorLeader.setControl(motionMagicControl
        .withPosition(position.in(Meters) * METERS_PER_REVOLUTION.in(Meters.per(Rotations)))
        .withLimitForwardMotion(isAtTopLimit())
        .withLimitReverseMotion(isAtBottomLimit()));
  }

  private void move(Measure<Voltage> volts) {
    elevatorLeader.setControl(voltageControl.withOutput(volts.in(Volts))
        .withLimitForwardMotion(isAtTopLimit())
        .withLimitReverseMotion(isAtBottomLimit()));
  }

  private void stop() {
    elevatorLeader.stopMotor();
  }

}
