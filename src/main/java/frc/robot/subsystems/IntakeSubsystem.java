// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is probably really janky, and it is untested on hardware but its fine I think

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IntakeConstants.DEPLOY_CANCODER_OFFSET;
import static frc.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_DEPLOYED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POSITION_RETRACTED;
import static frc.robot.Constants.IntakeConstants.DEPLOY_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_CANIVORE;
import static frc.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER;
import static frc.robot.Constants.IntakeConstants.ROLLER_INTAKE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_REVERSE_VELOCITY;
import static frc.robot.Constants.IntakeConstants.ROLLER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
  private final CANcoder deployCanCoder = new CANcoder(DEVICE_ID_DEPLOY_CANIVORE, CANIVORE_BUS_NAME);

  // Roller hardware
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER, CANIVORE_BUS_NAME);

  // Motor request objects
  private final MotionMagicTorqueCurrentFOC deployControl = new MotionMagicTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
  
  // Status signal objects for reading data from hardware
  private final StatusSignal<Double> deployPositionSignal;

  // SysId routines  
  private SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
    new SysIdRoutine.Mechanism((volts) -> deployMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));

  private SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> rollerMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));
    
  public IntakeSubsystem() {
    // Configure the deploy CANCoder
    var canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = DEPLOY_CANCODER_OFFSET;
    deployCanCoder.getConfigurator().apply(canCoderConfig);

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig.Feedback.RotorToSensorRatio = DEPLOY_ROTOR_TO_SENSOR_RATIO;
    deployConfig.Feedback.FeedbackRemoteSensorID = DEVICE_ID_DEPLOY_CANIVORE;
    deployConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
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
    rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_SENSOR_TO_MECHANISM_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);

    rollerMotor.getConfigurator().apply(rollerConfig);
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
   * Returns a new command to stop and retract the intake
   * @return new command
   */
  public Command retractIntakeCommand() {
    return runEnd(() -> {
          stopRoller();
          retractIntake();
        },
        this::stopAll).until(this::isRetracted);
  }

  /**
   * Returns a new command to deploy the intake
   * @return new command
   */
  public Command deployIntakeCommand() {
    return run(this::deployIntake);
  }

  /**
   * Returns a new command to run the intake rollers in the intake direction
   * @return new command
   */
  public Command runIntakeRollersCommand() {
    return run(this::rollerIntake).finallyDo(this::stopRoller);
  }

  /**
   * Returns a new command to run the intake rollers in the reverse (eject) direction
   * @return new command
   */
  public Command reverseIntakeRollersCommand() {
    return run(this::rollerReverse).finallyDo(this::stopRoller);
  }

  /**
   * Returns a new command to stop everything
   * @return new command
   */
  public Command stopAllCommand() {
    return runOnce(this::stopAll);
  }

  /**
   * Returns a new command that deploys and runs the intake
   * @return new command
   */
  public Command deployAndRunIntakeCommand() {
    return runEnd(() -> {
      deployIntakeCommand();
      runIntakeRollersCommand();
    },
    this::stopAll);
  }

  public Command sysIdDeployDynamicCommand(Direction direction) {
    return deploySysIdRoutine.dynamic(direction).withName("SysId intake deploy dynam " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdDeployQuasistaticCommand(Direction direction) {
    return deploySysIdRoutine.quasistatic(direction).withName("SysId intake deploy quasi " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId intake roller dynam " + direction)
        .finallyDo(this::stopAll);
  }

  public Command sysIdRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId intake roller quasi " + direction)
        .finallyDo(this::stopAll);
  }

  private void deployIntake() {
    deployMotor.setControl(deployControl.withPosition(DEPLOY_POSITION_DEPLOYED.in(Rotations)));
  }

  private void retractIntake() {
    deployMotor.setControl(deployControl.withPosition(DEPLOY_POSITION_RETRACTED.in(Rotations)));
  }

  private void rollerIntake() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_VELOCITY.in(RotationsPerSecond)));
  }

  private void rollerReverse() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_REVERSE_VELOCITY.in(RotationsPerSecond)));
  }

  private void stopRoller() {
    rollerMotor.stopMotor();
  }

  private void stopAll() {
    rollerMotor.stopMotor();
    deployMotor.stopMotor();
  }

}
