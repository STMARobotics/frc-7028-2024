// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TrapAmpShooterConstants.DEVICE_ID_TRAP_AMP_SHOOTER;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class AmpShooterSubsystem extends SubsystemBase {

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX ampShooterMotor = new TalonFX(DEVICE_ID_TRAP_AMP_SHOOTER);

  private SysIdRoutine ampMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        ampShooterMotor.setControl(voltageRequest.withOutput(volts.in((Volts))));
      }, null, this));

  public Command ampMotorQuasiCommand(Direction direction) {
    return ampMotorSysIdRoutine.quasistatic(direction).withName("SysId Amp Motors Quasistatic " + direction)
        .finallyDo(this::stopAmpShooter);
  }

  public Command ampMotorDynamCommand(Direction direction) {
    return ampMotorSysIdRoutine.dynamic(direction).withName("SysId Amp Motors Quasistatic " + direction)
        .finallyDo(this::stopAmpShooter);
  }

  private final VelocityTorqueCurrentFOC ampMotorVelocity = new VelocityTorqueCurrentFOC(
      0,
      0,
      0,
      1,
      false,
      false,
      false
      );

  public void runAmpShooter() {
    // velocity number is still a placeholder
    ampShooterMotor.setControl(ampMotorVelocity.withVelocity(10));
  }

  public void stopAmpShooter() {
    // velocity number is still a placeholder
    ampShooterMotor.stopMotor();
  }
}
