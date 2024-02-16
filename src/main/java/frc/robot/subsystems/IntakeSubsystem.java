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

import java.util.function.Consumer;

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
import edu.wpi.first.units.MutableMeasure;
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

  private final MutableMeasure<Angle> telemetryDeployAngle = MutableMeasure.zero(Rotations);
  private final Consumer<Measure<Angle>> telemetryConsumer;

  private boolean encoderSynced = false;

  // SysId routines  
  private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> deployMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));

  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> rollerMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));
    
  public IntakeSubsystem(Consumer<Measure<Angle>> telemetryConsumer) {
    this.telemetryConsumer = telemetryConsumer;



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
    if (telemetryConsumer != null) {
      telemetryConsumer.accept(getPosition());
    }
    if (!encoderSynced && deployEncoder.isConnected()) {
      // Configure the deploy encoder
      deployEncoder.setPositionOffset(DEPLOY_ENCODER_OFFSET.in(Rotations));
      encoderSynced = deployMotor.setPosition(getPosition().in(Rotations)) == StatusCode.OK
          &&  Math.abs(getPosition().in(Rotations) - deployPositionSignal.refresh().getValueAsDouble()) < .05;
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
   * Returns a new command to run the intake rollers
   * @param velocity target velocity
   * @return new command
   */
  public Command runIntakeRollersCommand(Measure<Velocity<Angle>> velocity) {
    return run(() -> this.rollerIntake(velocity)).finallyDo(this::stopRoller);
  }

  /**
   * Returns a new command to set the intake position
   * @param position target position
   * @return new command
   */
  public Command setIntakePositionCommand(Measure<Angle> position) {
    return run(() -> this.setIntakePosition(position)).finallyDo(this::stopDeploy);
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
   * Returns a new command that deploys and runs the rollers. Retracts the intake and stops the rollers when cancelled.
   * @return new command
   */
  public Command deployAndRunIntakeCommand() {
    return runEnd(() -> {
      deployIntake();
      rollerIntake();
    }, () -> {
      stopRoller();
      retractIntake();
    });
  }

  /**
   * Returns a new command that deploys and runs the rollers in reverse. Retracts the intake and stops the rollers when
   * cancelled.
   * @return new command
   */
  public Command deployAndReverseIntakeCommand() {
    return runEnd(() -> {
      deployIntake();
      rollerReverse();
    }, () -> {
      stopRoller();
      retractIntake();
    });
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

  private Measure<Angle> getPosition() {
    var position = -deployEncoder.get();
    position = IEEEremainder(position, 1.0);
    return telemetryDeployAngle.mut_replace(position, Rotations);
  }

  private void deployIntake() {
    setIntakePosition(DEPLOY_POSITION_DEPLOYED);
  }

  private void retractIntake() {
    setIntakePosition(DEPLOY_POSITION_RETRACTED);
  }

  private void setIntakePosition(Measure<Angle> position) {
    deployMotor.setControl(deployControl.withPosition(position.in(Rotations)));
  }

  private void rollerIntake() {
    rollerIntake(ROLLER_INTAKE_VELOCITY);
  }

  private void rollerIntake(Measure<Velocity<Angle>> velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  private void rollerReverse() {
    rollerMotor.setControl(rollerControl.withVelocity(ROLLER_REVERSE_VELOCITY.in(RotationsPerSecond)));
  }

  private void stopRoller() {
    rollerMotor.stopMotor();
  }

  private void stopDeploy() {
    deployMotor.stopMotor();
  }

  private void stopAll() {
    rollerMotor.stopMotor();
    deployMotor.stopMotor();
  }

}
