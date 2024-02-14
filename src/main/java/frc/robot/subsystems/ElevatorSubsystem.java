
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.CHAIN_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;
import static frc.robot.Constants.ElevatorConstants.MOTOR_BOTTOM;
import static frc.robot.Constants.ElevatorConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;
import static frc.robot.Constants.ElevatorConstants.MOTOR_TOP;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // Mutiply by sensor position to get meters

  private final TalonFX elevatorLeader;
  private final TalonFX elevatorFollower;
  private final AnalogInput analogSensor;

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private double targetPosition = 0;

  public ElevatorSubsystem() {
    elevatorLeader = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID);
    elevatorFollower = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID);

    // Configure potentiometer
    analogSensor = new AnalogInput(ElevatorConstants.ANALOG_SENSOR_CHANNEL);

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    elevatorLeader.getConfigurator().apply(config);
    elevatorFollower.getConfigurator().apply(config);

    elevatorLeader.setInverted(true);
    elevatorFollower.setInverted(true);

    // Brake mode helps hold the elevator in place
    elevatorLeader.setNeutralMode(null);
    elevatorFollower.setNeutralMode(null);

    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), false));
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
  
}