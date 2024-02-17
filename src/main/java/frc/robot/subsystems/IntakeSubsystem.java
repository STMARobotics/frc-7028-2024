// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//still untested on hardware but I did update this so that intake should work with Falcon motors -eleanor :)

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLERS_MOTOR;
import static frc.robot.Constants.IntakeConstants.MagnetOffsetValue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class IntakeSubsystem extends SubsystemBase {

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX rollersMotor = new TalonFX(DEVICE_ID_ROLLERS_MOTOR);

  private SysIdRoutine intakeMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        rollersMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  public Command intakeMotorQuasiCommand(Direction direction) {
    return intakeMotorSysIdRoutine.quasistatic(direction).withName("SysId Amp Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command intakeMotorDynamCommand(Direction direction) {
    return intakeMotorSysIdRoutine.dynamic(direction).withName("SysId Amp Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public IntakeSubsystem() {
    var intakeRollersConfig = new TalonFXConfiguration();
    intakeRollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollersMotor.getConfigurator().apply(intakeRollersConfig);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = MagnetOffsetValue;
  }

  private final VelocityTorqueCurrentFOC intakeRollersMotorVelocity = new VelocityTorqueCurrentFOC(
      0,
      0,
      0,
      1,
      false,
      false,
      false
      );

  public void runintakeRollers() {
    rollersMotor.setControl(intakeRollersMotorVelocity.withVelocity(10));
  }

  public void reverseintakeRollers() {
    rollersMotor.setControl(intakeRollersMotorVelocity.withVelocity(-10));
  }

  public void stop() {
    // placeholder motor velocity
    rollersMotor.stopMotor();
  }

}
