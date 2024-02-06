// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_PARK_HEIGHT;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ElevatorSubsystem extends SubsystemBase {

  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final TalonFX elevatorLeader = new TalonFX(ElevatorConstants.DEVICE_ID_MOTOR_1, CANIVORE_BUS_NAME);
  private final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.DEVICE_ID_MOTOR_2, CANIVORE_BUS_NAME);

  private SysIdRoutine elavatorSysidRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        elevatorLeader.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  // Elevator travel distance, in meters
  private static final double ELEVATOR_HEIGHT = 1.002;

  // Motor's encoder limits, in encoder ticks
  private static final double MOTOR_BOTTOM = 0;
  private static final double MOTOR_TOP = 56530;

  // Mutiply by sensor position to get meters
  private static final double MOTOR_ENCODER_POSITION_COEFFICIENT = ELEVATOR_HEIGHT / (MOTOR_TOP - MOTOR_BOTTOM);

  private static final int ANALOG_BOTTOM = 758;
  private static final int ANALOG_TOP = 1796;

  private static final double GRAVITY_FEED_FORWARD = 0.05;

  // Mutiply by sensor position to get meters
  private static final double ANALOG_SENSOR_COEFFICIENT = ELEVATOR_HEIGHT / (ANALOG_TOP - ANALOG_BOTTOM);


  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(8);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(9);

  private double targetPosition = 0;

  public ElevatorSubsystem() {
    elevatorLeader.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollower.getConfigurator().apply(new TalonFXConfiguration());

    // Configure closed-loop control
    double kP = 0.13;
    double kI = 0;
    double kD = 0;
    double kV = 0.12;
    double kIz = 0;
    double kS = 0.25;
    double kMaxOutput = 0.7;
    double kMinOutput = -.4;
    double allowedErr = 1;

    // Magic Motion Coefficients
    double maxVel = 10000;
    double maxAcc = 30000;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorLeader.getConfigurator().apply(config);
    elevatorFollower.getConfigurator().apply(config);

    elevatorLeader.getConfigurator().refresh(config);
    elevatorFollower.getConfigurator().refresh(config);

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

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
  }

  public void moveElevator(double speed) {
    targetPosition = 0;
    elevatorLeader.set(speed);
  }

  public void moveToPosition(double meters) {
    targetPosition = meters;
    elevatorLeader.set(meters);
  }

  public Command sysIdElevatorQuasiCommand(Direction direction) {
    return elavatorSysidRoutine.quasistatic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdElevatorDynamCommand(Direction direction) {
    return elavatorSysidRoutine.dynamic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public double getElevatorPosition() {
    var rotorPosSignal = elevatorLeader.getRotorPosition();
    var rotorPos = rotorPosSignal.getValue();
    rotorPosSignal.waitForUpdate(0.020);
    return rotorPos;
  }

  public double getElevatorTopPosition() {
    return getElevatorPosition() * .489;
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

  static double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  static double metersToMotorPosition(double positionMeters) {
    return (positionMeters / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  private static double kDt = 0.02;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_feedforward = new TrapezoidProfile.State();

}