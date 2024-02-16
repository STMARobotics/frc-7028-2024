// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TrapAmpShooterConstants.DEVICE_ID_TRAP_AMP_SHOOTER;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpShooterSubsystem extends SubsystemBase {

  private final VelocityTorqueCurrentFOC ampMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);

  // device id is a placeholder!!!
  private final TalonFX ampShooterMotor = new TalonFX(DEVICE_ID_TRAP_AMP_SHOOTER);

  public void runAmpShooter() {
    // velocity number is still a placeholder
    ampShooterMotor.setControl(ampMotorVelocity.withVelocity(10));
  }

  public void stopAmpShooter() {
    // velocity number is still a placeholder
    ampShooterMotor.stopMotor();
  }
}
