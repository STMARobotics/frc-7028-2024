// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ElevatorConstants.BOTTOM_LIMIT;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_MOTOR;
import static frc.robot.Constants.ElevatorConstants.DEVICE_PORT_BOTTOM_LIMIT;
import static frc.robot.Constants.ElevatorConstants.DEVICE_PORT_TOP_LIMIT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_TRANSFER_TO_AMP_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.METERS_PER_ROTATION;
import static frc.robot.Constants.ElevatorConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.POSITION_TOLERANCE;
import static frc.robot.Constants.ElevatorConstants.SLOT_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.TOP_LIMIT;

import java.util.function.Consumer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;
import frc.robot.telemetry.ElevatorState;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(DEVICE_ID_MOTOR, CANIVORE_BUS_NAME);

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(DEVICE_PORT_TOP_LIMIT);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(DEVICE_PORT_BOTTOM_LIMIT);

  private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);

  private final ElevatorState elevatorState = new ElevatorState();
  private final Consumer<ElevatorState> telemetryFunction;
  private final StatusSignal<Double> elevatorPositionSignal;

  // SysId routines  
  private final SysIdRoutine elevatorRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(1.0), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> 
          elevatorMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));
  
  public ElevatorSubsystem(Consumer<ElevatorState> telemetryFunction) {
    this.telemetryFunction = telemetryFunction;

    var motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    motorConfig.MotionMagic = MOTION_MAGIC_CONFIGS;
    motorConfig.MotorOutput.NeutralMode = Brake;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TOP_LIMIT.in(Meters) / METERS_PER_ROTATION.in(Meters.per(Rotations));
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        BOTTOM_LIMIT.in(Meters) / METERS_PER_ROTATION.in(Meters.per(Rotations));
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevatorMotor.getConfigurator().apply(motorConfig);

    elevatorPositionSignal = elevatorMotor.getPosition();
  }

  @Override
  public void periodic() {
    elevatorState.isAtBottomLimit = isAtBottomLimit();
    elevatorState.isAtTopLimit = isAtTopLimit();
    elevatorState.elevatorMeters = getPositionMeters();
    if (telemetryFunction != null) {
      telemetryFunction.accept(elevatorState);
    }
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
   * Moves the elevator to position to score in the amp
   */
  public void prepareToAmp() {
    moveToPosition(ElevatorConstants.SCORE_AMP_HEIGHT);
  }

  /**
   * Moves the elevator to the position to score in the trap
   */
  public void prepareToTrap() {
    moveToPosition(ElevatorConstants.SCORE_TRAP_HEIGHT);
  }

  /**
   * Moves the elevator to position to exchange from turret to amper
   */
  public void prepareToExchangeToAmper() {
    moveToPosition(ELEVATOR_TRANSFER_TO_AMP_HEIGHT);
  }

  /**
   * Moves the elevator to the park height
   */
  public void park() {
    moveToPosition(ElevatorConstants.PARK_HEIGHT);
  }

  /**
   * Checks if the elevator is at the target position
   * @return true if the elevator is at the target, otherwise false
   */
  public boolean isAtTarget() {
    return rotationsToMeters(Math.abs(
        elevatorPositionSignal.refresh().getValueAsDouble() - motionMagicControl.Position)) < POSITION_TOLERANCE.in(Meters);
  }

  /**
   * Gets the height of the elevator
   * @return height of the elevator in meters
   */
  private double getPositionMeters() {
    return rotationsToMeters(elevatorPositionSignal.refresh().getValueAsDouble());
  }

  public Command sysIdDynamicCommand(Direction direction) {
    return elevatorRoutine.dynamic(direction).withName("SysId elevator dynamic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdQuasistaticCommand(Direction direction) {
    return elevatorRoutine.quasistatic(direction).withName("SysId elevator quasistatic " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Moves the elevator to a position
   * @param height height to move to
   */
  public void moveToPosition(Measure<Distance> height) {
    elevatorMotor.setControl(motionMagicControl
        .withPosition(heightToRotations(height))
        .withLimitForwardMotion(isAtTopLimit())
        .withLimitReverseMotion(isAtBottomLimit()));
  }

  private double rotationsToMeters(double rotations) {
    return rotations * METERS_PER_ROTATION.in(Meters.per(Rotations));
  }

  private double heightToRotations(Measure<Distance> height) {
    return height.in(Meters) / METERS_PER_ROTATION.in(Meters.per(Rotations));
  }

  public void stop() {
    elevatorMotor.stopMotor();
  }

}
