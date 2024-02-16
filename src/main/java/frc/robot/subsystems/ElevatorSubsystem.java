
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimbConstants.CHAIN_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SLOT_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.MOTOR_BOTTOM;
import static frc.robot.Constants.ElevatorConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;
import static frc.robot.Constants.ElevatorConstants.MOTOR_TOP;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ElevatorSubsystem extends SubsystemBase {

  // Mutiply by sensor position to get meters

  private final TalonFX elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID);
  private final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);;
  private final AnalogInput analogSensor = new AnalogInput(ElevatorConstants.ANALOG_SENSOR_CHANNEL);;

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private double targetPosition = 0;

  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

private SysIdRoutine elevatorLeaderSysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
  new SysIdRoutine.Mechanism((volts) -> {
    elevatorLeader.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }, null, this));

private SysIdRoutine elevatorFollowerSysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
  new SysIdRoutine.Mechanism((volts) -> {
    elevatorFollower.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }, null, this));

  public ElevatorSubsystem() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.Slot0 = Slot0Configs.from(ELEVATOR_SLOT_CONFIGS);
    config.MotorOutput.NeutralMode = Brake;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;

    elevatorLeader.getConfigurator().apply(config);
    elevatorFollower.getConfigurator().apply(config);
    
    elevatorLeader.setInverted(true);
    elevatorFollower.setInverted(false);
  }

  @Override
  public void periodic() {
    // Handle elevator limit switches
    if (isAtBottomLimit()) {
      elevatorLeader.setPosition(MOTOR_BOTTOM);
    } else if (isAtTopLimit()) {
      elevatorLeader.setPosition(MOTOR_TOP);
    }
  }

  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorLeader.set(speed);
  }

  public void moveToPosition(double meters) {
    targetPosition = metersToMotorPosition(meters);
    elevatorLeader.set(targetPosition);
  }

  public void climbUp() {
    elevatorLeader.set(metersToMotorPosition(CHAIN_HEIGHT));
  }

  public void stopClimb() {
    elevatorLeader.stopMotor();
  }

  public double getElevatorPosition() {
    var elevatorPositionSignal = elevatorLeader.getPosition();
    var elevatorPosition = elevatorPositionSignal.getValue();
    return motorPositionToMeters(elevatorPosition);
  }

  public void parkElevator() {
    moveToPosition(ELEVATOR_PARK_HEIGHT);
  }

  public boolean isParked() {
    return getElevatorPosition() < (ELEVATOR_PARK_HEIGHT + 0.1);
  }

  public void stop() {
    elevatorLeader.stopMotor();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  private static double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  private static double metersToMotorPosition(double positionMeters) {
    return (positionMeters / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  public Command sysIdElevatorLeaderQuasiCommand(Direction direction) {
  return elevatorLeaderSysIdRoutine.quasistatic(direction).withName("SysId Elevator Motors Quasistatic " + direction)
      .finallyDo(this::stop);
  }
  
  public Command sysIdElevatorFollowerQuasiCommand(Direction direction) {
    return elevatorFollowerSysIdRoutine.quasistatic(direction).withName("SysId Elevator Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdElevatorLeaderDynamCommand(Direction direction) {
    return elevatorLeaderSysIdRoutine.dynamic(direction).withName("SysId Elevator Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdElevatorFollowerDynamCommand(Direction direction) {
    return elevatorFollowerSysIdRoutine.dynamic(direction).withName("SysId Elevator Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }
  
}
