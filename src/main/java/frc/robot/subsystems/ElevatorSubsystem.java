
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.BOTTOM_LIMIT_SWITCH_CHANNEL;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SLOT_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.MOTOR_BOTTOM;
import static frc.robot.Constants.ElevatorConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;
import static frc.robot.Constants.ElevatorConstants.MOTOR_TOP;
import static frc.robot.Constants.ElevatorConstants.TOP_LIMIT_SWITCH_CHANNEL;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ElevatorSubsystem extends SubsystemBase {

  // Mutiply by sensor position to get meters

  private final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_POSITON_ID);

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_CHANNEL);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_CHANNEL);

  private double targetPosition = 0;

  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

private SysIdRoutine elevatorMotorSysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
  new SysIdRoutine.Mechanism((volts) -> {
    elevatorMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }, null, this));

  public ElevatorSubsystem() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.Slot0 = Slot0Configs.from(ELEVATOR_SLOT_CONFIGS);
    config.MotorOutput.NeutralMode = Brake;

    elevatorMotor.getConfigurator().apply(config);

  }

  @Override
  public void periodic() {
    // Handle elevator limit switches
    if (isAtBottomLimit()) {
      elevatorMotor.setPosition(MOTOR_BOTTOM);
    } else if (isAtTopLimit()) {
      elevatorMotor.setPosition(MOTOR_TOP);
    }
  }

  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorMotor.set(speed);
  }

  public void moveToPosition(double meters) {
    targetPosition = metersToMotorPosition(meters);
    elevatorMotor.setPosition(targetPosition);
  }

  public double getElevatorPosition() {
    var elevatorPositionSignal = elevatorMotor.getPosition();
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
    elevatorMotor.stopMotor();
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

  public Command sysIdelevatorMotorQuasiCommand(Direction direction) {
  return elevatorMotorSysIdRoutine.quasistatic(direction).withName("SysId Elevator Motor Quasistatic " + direction)
      .finallyDo(this::stop);
  }

  public Command sysIdelevatorMotorDynamCommand(Direction direction) {
    return elevatorMotorSysIdRoutine.dynamic(direction).withName("SysId Elevator Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }
  
}
