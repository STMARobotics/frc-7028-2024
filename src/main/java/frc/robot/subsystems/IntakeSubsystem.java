// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//still untested on hardware but I did update this so that intake should work with Falcon motors -eleanor :)

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;



public class IntakeSubsystem extends SubsystemBase {
  
  private VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX IntakeMotor = new TalonFX(1);

  private SysIdRoutine intakeMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> 
        IntakeMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null, this));

  public IntakeSubsystem() {
  var intakeMotorConfig = new TalonFXConfiguration();
  intakeMotorConfig.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
  intakeMotorConfig.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
  intakeMotorConfig.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  IntakeMotor.getConfigurator().apply(intakeMotorConfig);  
}

  private final VelocityTorqueCurrentFOC IntakerMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command Intake() {
    return runOnce(
        () -> {
          //READ THIS!!!!!
          //IDK what the motor velocity number should actually be, just set it to 1 for now
          IntakeMotor.setControl(IntakerMotorVelocity.withVelocity(1));
        });
  }

  //Spit means to reverse the motor direction to 'spit' the game piece out 
  /**
   * @return
   */
  public Command Spit() {
    return runOnce(
        () -> {
          //READ THIS!!!!!
          //IDK what the motor velocity number should actually be, just set it to -1 for now
          IntakeMotor.setControl(IntakerMotorVelocity.withVelocity(-1));
        });
      }
      public Command sysIdIntakeMotorQuasiCommand(Direction direction) {
        return intakeMotorSysIdRoutine.quasistatic(direction).withName("SysId Drive Quasistatic " + direction)
            .finallyDo(this::Stop);
      }
    
      public Command sysIdIntakeMotorDynamCommand(Direction direction) {
        return intakeMotorSysIdRoutine.dynamic(direction).withName("SysId Drive Quasistatic " + direction)
            .finallyDo(this::Stop);
      }


  public Command Stop() {
    return runOnce(
        () -> {
          //READ THIS!!!!!
          //IDK what the motor velocity number should actually be, just set it to 0 for now
          IntakeMotor.setControl(IntakerMotorVelocity.withVelocity(0));
        });

  //Cringe example things I think I should keep
  ///**
  // * An example method querying a boolean state of the subsystem (for example, a digital sensor).
  // *
  // * @return value of some boolean subsystem state, such as a digital sensor.
  // */
  //public boolean exampleCondition() {
  //  // Query some boolean state, such as a digital sensor.
  //  return false;
  //}
//
  //@Override
  //public void periodic() {
  //  // This method will be called once per scheduler run
  //}
//
  //@Override
  //public void simulationPeriodic() {
  //  // This method will be called once per scheduler run during simulation
  }
}