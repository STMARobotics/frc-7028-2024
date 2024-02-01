// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Meters;
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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor0 = new TalonFX(DEVICE_ID_MOTOR_0, CANIVORE_BUS_NAME);
  private final TalonFX elevatorMotor1 = new TalonFX(DEVICE_ID_MOTOR_1, CANIVORE_BUS_NAME);

  // Limit switches - FALSE means at limit
  private final DigitalInput topLimitSwitch = new DigitalInput(DEVICE_PORT_TOP_LIMIT);
  private final DigitalInput bottomLimitSwitch = new DigitalInput(DEVICE_PORT_BOTTOM_LIMIT);

  private final VoltageOut voltageControl = new VoltageOut(0);
  private final MotionMagicTorqueCurrentFOC motionMagicControl = new MotionMagicTorqueCurrentFOC(0);

  // SysId routines  
  private SysIdRoutine elevatorRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        elevatorMotor0.setControl(voltageControl.withOutput(volts.in(Volts)));
        elevatorMotor1.setControl(voltageControl.withOutput(volts.in(Volts)));
      }, null, this));
  
  public ElevatorSubsystem() {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    motorConfig.MotionMagic = MOTION_MAGIC_CONFIGS;
    motorConfig.MotorOutput.NeutralMode = Brake;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TOP_LIMIT.in(Meters) / METERS_PER_REVOLUTION;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BOTTOM_LIMIT.in(Meters) / METERS_PER_REVOLUTION;

    elevatorMotor0.getConfigurator().apply(motorConfig);
    elevatorMotor1.getConfigurator().apply(motorConfig);

    elevatorMotor1.setControl(new Follower(elevatorMotor0.getDeviceID(), false));
  }

  public boolean isAtTopLimit() {
    return !topLimitSwitch.get();
  }

  public boolean isAtBottomLimit() {
    return !bottomLimitSwitch.get();
  }

  public Command moveToPositionCommand(Measure<Distance> position) {
    return run(() -> this.moveToPosition(position)).finallyDo(this::stop);
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
    elevatorMotor0.setControl(motionMagicControl
        .withPosition(position.in(Meters) * METERS_PER_REVOLUTION)
        .withLimitForwardMotion(isAtTopLimit())
        .withLimitReverseMotion(isAtBottomLimit()));
  }

  private void stop() {
    elevatorMotor0.stopMotor();
  }

}
