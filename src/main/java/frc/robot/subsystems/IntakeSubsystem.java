// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is probably really janky, and it is untested on hardware but its fine I think

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(1, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.setOpenLoopRampRate(.1);
    intakeMotor.burnFlash();
  }

  public void intake() {
    intakeMotor.set(1d);
  }

  public void reverse() {
    intakeMotor.set(-1d);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public Command runIntakeMotorCommand() {
    return run(this::intake).finallyDo(this::stop);
  }

  public Command reverseIntakeMotorCommand() {
    return run(this::reverse).finallyDo(this::stop);
  }

  public Command stopMotorCommand() {
    return runOnce(this::stop);
  }

}
