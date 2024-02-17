// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is probably really janky, and it is untested on hardware but its fine I think

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IntakeConstants.DEPLOY_ENCODER_OFFSET;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_ENCODER;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER;
import static frc.robot.Constants.IntakeConstants.ROLLER_INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;
import static java.lang.Math.IEEEremainder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/**
 * Subsystem for the intake. The intake is controlled by two TalonFX motor controllers, one to deploy the intake, and
 * the other to run the rollers.
 */
public class IntakeSubsystem extends SubsystemBase {

  // Deploy hardware
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY, CANIVORE_BUS_NAME);
  private final DutyCycleEncoder deployEncoder = new DutyCycleEncoder(DEVICE_ID_DEPLOY_ENCODER);

  // Roller hardware
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER, CANIVORE_BUS_NAME);

  // Motor request objects
  private final MotionMagicVoltage deployControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
  
  // Status signal objects for reading data from hardware
  private final StatusSignal<Double> deployPositionSignal;

  private boolean encoderSynced = false;

  // SysId routines  
  private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> deployMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));

  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> rollerMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));
    
  public IntakeSubsystem() {
    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deployConfig.MotorOutput.NeutralMode = Brake;
    deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_SENSOR_TO_MECHANISM_RATIO;
    deployConfig.Slot0 = Slot0Configs.from(DEPLOY_SLOT_CONFIGS);
    deployConfig.MotionMagic = DEPLOY_MOTION_MAGIC_CONFIGS;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITION_RETRACTED.in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION_DEPLOYED.in(Rotations);
    deployMotor.getConfigurator().apply(deployConfig);

    deployPositionSignal = deployMotor.getPosition();

    // Configure the roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_SENSOR_TO_MECHANISM_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void periodic() {
    if (!encoderSynced && deployEncoder.isConnected()) {
      // Configure the deploy encoder
      deployEncoder.setPositionOffset(DEPLOY_ENCODER_OFFSET.in(Rotations));
      encoderSynced = deployMotor.setPosition(getPosition()) == StatusCode.OK
          &&  Math.abs(getPosition() - deployPositionSignal.refresh().getValueAsDouble()) < .05;
    }
  }

  /**
   * Returns true if the intake is in the retract position
   * @return true if the intake is in the retract position, otherwise false
   */
  public boolean isRetracted() {
    return Math.abs(deployPositionSignal.refresh().getValueAsDouble() - DEPLOY_POSITION_RETRACTED.in(Rotations))
       <= DEPLOY_TOLERANCE.in(Rotations);
  }

  /**
   * Returns true if the intake is in the deployed position
   * @return true if the intake is in the deployed position, otherwise false
   */
  public boolean isDeployed() {
    return Math.abs(deployPositionSignal.refresh().getValueAsDouble() - DEPLOY_POSITION_DEPLOYED.in(Rotations))
       <= DEPLOY_TOLERANCE.in(Rotations);
  }

  public Command sysIdDeployMotorDynamCommand(Direction direction) {
    return deploySysIdRoutine.dynamic(direction).withName("SysId intake deploy dynam " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdDeployMotorQuasiCommand(Direction direction) {
    return deploySysIdRoutine.quasistatic(direction).withName("SysId intake deploy quasi " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdRollersMotorDynamCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId intake roller dynam " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdRollersMotorQuasiCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId intake roller quasi " + direction)
        .finallyDo(this::stopAll);
  }

  private double getPosition() {
    var position = -deployEncoder.get();
    position = IEEEremainder(position, 1.0);
    return position;
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

  public void rollers() {
    rollerIntake(ROLLER_INTAKE_VELOCITY);
  }

  private void rollerIntake(Measure<Velocity<Angle>> velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  public void rollerReverse() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_REVERSE_VELOCITY.in(RotationsPerSecond)));
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  public void stopAll() {
    rollerMotor.stopMotor();
    deployMotor.stopMotor();
  }

}
