// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//still untested on hardware but I did update this so that intake should work with Falcon motors -eleanor :)

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IntakeConstants.DEPLOY_CANCODER_OFFSET;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_CANCODER;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_MOTOR;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLERS_MOTOR;
import static frc.robot.Constants.IntakeConstants.ROLLERS_SLOT_CONFIGS;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class IntakeSubsystem extends SubsystemBase {

  private VoltageOut voltageRequest = new VoltageOut(0);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY_MOTOR, CANIVORE_BUS_NAME);
  private final TalonFX rollersMotor = new TalonFX(DEVICE_ID_ROLLERS_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder canCoder = new CANcoder(DEVICE_ID_DEPLOY_CANCODER, CANIVORE_BUS_NAME);

  private SysIdRoutine deployMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> deployMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null,
          this));

  private SysIdRoutine rollersMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> rollersMotor.setControl(voltageRequest.withOutput(volts.in(Volts))), null,
          this));

  public IntakeSubsystem() {
    var intakeRollersConfig = new TalonFXConfiguration();
    
    var intakeDeployConfig = new TalonFXConfiguration();
    intakeRollersConfig.Slot0 = Slot0Configs.from(ROLLERS_SLOT_CONFIGS);
    intakeRollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollersMotor.getConfigurator().apply(intakeRollersConfig);
    
    intakeDeployConfig.Slot0 = Slot0Configs.from(DEPLOY_SLOT_CONFIGS);
    intakeDeployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeDeployConfig.Feedback.RotorToSensorRatio = DEPLOY_MOTOR_TO_SENSOR_RATIO;
    intakeDeployConfig.Feedback.FeedbackRemoteSensorID = DEVICE_ID_DEPLOY_CANCODER;
    intakeDeployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    intakeDeployConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    intakeDeployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    intakeDeployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITION_RETRACTED.in(Rotations);
    intakeDeployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    intakeDeployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    deployMotor.getConfigurator().apply(intakeDeployConfig);
    
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = DEPLOY_CANCODER_OFFSET.in(Rotations);
    canCoder.getConfigurator().apply(canCoderConfig);
  }

  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final VelocityTorqueCurrentFOC IntakeRollersMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false,
      false).withSlot(0);

  private final VelocityTorqueCurrentFOC IntakeDeployMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false,
      false);

  public void rollers() {
    rollersMotor.setControl(IntakeRollersMotorVelocity.withVelocity(50));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void intakeRollers(Measure<Voltage> volts) {
    rollersMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
  }

  // Spit means to reverse the motor direction to 'spit' the game piece out
  /**
   * @return
   */
  public void spit(double rps) {
    deployMotor.setControl(IntakeDeployMotorVelocity.withVelocity(-rps));
  }

  public Command sysIdDeployMotorQuasiCommand(Direction direction) {
    return deployMotorSysIdRoutine.quasistatic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stopDeploy);
  }

  public Command sysIdDeployMotorDynamCommand(Direction direction) {
    return deployMotorSysIdRoutine.dynamic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stopDeploy);
  }

  public Command sysIdRollersMotorQuasiCommand(Direction direction) {
    return rollersMotorSysIdRoutine.quasistatic(direction).withName("SysId Rollers Motor Quasistatic " + direction)
        .finallyDo(this::stopRollers);
  }

  public Command sysIdRollersMotorDynamCommand(Direction direction) {
    return rollersMotorSysIdRoutine.dynamic(direction).withName("SysId Rollers Motor Quasistatic " + direction)
        .finallyDo(this::stopRollers);
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  public void stopRollers() {
    rollersMotor.stopMotor();
  }

  public void deployIntake() {
    setIntakePosition(DEPLOY_POSITION_DEPLOYED);
  }

  public void retractIntake() {
    setIntakePosition(DEPLOY_POSITION_RETRACTED);
  }

  private void setIntakePosition(Measure<Angle> position) {
    deployMotor.setControl(deployControl.withPosition(position.in(Rotations)));
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
