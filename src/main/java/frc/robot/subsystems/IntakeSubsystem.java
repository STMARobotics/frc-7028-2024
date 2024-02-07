// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//still untested on hardware but I did update this so that intake should work with Falcon motors -eleanor :)

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_CANCODER;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_MOTOR;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLERS_MOTOR;
import static frc.robot.Constants.IntakeConstants.ROLLERS_SLOT_CONFIGS;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class IntakeSubsystem extends SubsystemBase {

  private VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY_MOTOR, CANIVORE_BUS_NAME);
  private final TalonFX RollersMotor = new TalonFX(DEVICE_ID_ROLLERS_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder canCoder = new CANcoder(DEVICE_ID_DEPLOY_CANCODER, CANIVORE_BUS_NAME);

  private SysIdRoutine deployMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> deployMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null,
          this));

  private SysIdRoutine rollersMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> RollersMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null,
          this));

  public IntakeSubsystem() {
    var intakeRollersConfig = new TalonFXConfiguration();
    var intakeDeployConfig = new TalonFXConfiguration();
    intakeRollersConfig.Slot0 = Slot0Configs.from(ROLLERS_SLOT_CONFIGS);
    RollersMotor.getConfigurator().apply(intakeRollersConfig);
    intakeDeployConfig.Slot0 = Slot0Configs.from(DEPLOY_SLOT_CONFIGS);
    deployMotor.getConfigurator().apply(intakeDeployConfig);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = 0.4;
    canCoder.getConfigurator().apply(canCoderConfig);
  }

  private final VelocityTorqueCurrentFOC IntakeRollersMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false,
      false);

  private final VelocityTorqueCurrentFOC IntakeDeployMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false,
      false);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeRollers() {
    return runOnce(
        () -> {
          // READ THIS!!!!!
          // IDK what the motor velocity number should actually be, just set it to 1 for
          // now
          deployMotor.setControl(IntakeRollersMotorVelocity.withVelocity(1));
        });
  }

  public void deploy(double rps) {
    deployMotor.setControl(IntakeDeployMotorVelocity.withVelocity(rps));
  }

  // Spit means to reverse the motor direction to 'spit' the game piece out
  /**
   * @return
   */
  public void spit(double rps) {
    deployMotor.setControl(IntakeRollersMotorVelocity.withVelocity(0 - rps));
  }

  public Command sysIdDeployMotorQuasiCommand(Direction direction) {
    return deployMotorSysIdRoutine.quasistatic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdDeployMotorDynamCommand(Direction direction) {
    return deployMotorSysIdRoutine.dynamic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdRollersMotorQuasiCommand(Direction direction) {
    return rollersMotorSysIdRoutine.quasistatic(direction).withName("SysId Rollers Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdRollersMotorDynamCommand(Direction direction) {
    return rollersMotorSysIdRoutine.dynamic(direction).withName("SysId Rollers Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public void stop() {
    // READ THIS!!!!!
    // IDK what the motor velocity number should actually be, just set it to 0 for
    // now
    deployMotor.setControl(IntakeDeployMotorVelocity.withVelocity(0));
    RollersMotor.setControl(IntakeRollersMotorVelocity.withVelocity(0));
  }

  // Cringe example things I think I should keep
  /// **
  // * An example method querying a boolean state of the subsystem (for example, a
  // digital sensor).
  // *
  // * @return value of some boolean subsystem state, such as a digital sensor.
  // */
  // public boolean exampleCondition() {
  // // Query some boolean state, such as a digital sensor.
  // return false;
  // }
  //
  // @Override
  // public void periodic() {
  // // This method will be called once per scheduler run
  // }
  //
  // @Override
  // public void simulationPeriodic() {
  // // This method will be called once per scheduler run during simulation
}
