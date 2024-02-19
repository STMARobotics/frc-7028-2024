package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_PITCH_ENCODER;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_PITCH_MOTOR;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_ROLLER_MOTOR;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_YAW_ENCODER;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_YAW_MOTOR;
import static frc.robot.Constants.TurretConstants.EJECT_VELOCITY;
import static frc.robot.Constants.TurretConstants.INTAKE_PITCH_POSITION;
import static frc.robot.Constants.TurretConstants.INTAKE_YAW_POSITION;
import static frc.robot.Constants.TurretConstants.LOAD_VELOCITY;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.TurretConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_TOLERANCE;
import static frc.robot.Constants.TurretConstants.ROLLER_VELOCITY_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.SHOOT_VELOCITY;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_MAGNETIC_OFFSET;
import static frc.robot.Constants.TurretConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_TOLERANCE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX yawMotor = new TalonFX(DEVICE_ID_YAW_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder yawEncoder = new CANcoder(DEVICE_ID_YAW_ENCODER, CANIVORE_BUS_NAME);

  private final TalonFX pitchMotor = new TalonFX(DEVICE_ID_PITCH_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(DEVICE_ID_PITCH_ENCODER, CANIVORE_BUS_NAME);

  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER_MOTOR, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

  private final TorqueCurrentFOC rollerSysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Double> yawPositionSignal;
  private final StatusSignal<Double> pitchPositionSignal;
  
  // SysId routines
  private final SysIdRoutine yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4.0), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> 
          yawMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));
  
  private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2.0), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> 
          pitchMotor.setControl(voltageControl.withOutput(volts.in(Volts))), null, this));

  // NOTE: the output type for the rollers is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(5).per(Seconds.of(1)), Volts.of(30), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((amps) -> rollerMotor.setControl(rollerSysIdControl.withOutput(amps.in(Volts))), null, this));

  public TurretSubsystem() {
    // Configure the yaw CANCoder
    var yawCanCoderConfig = new CANcoderConfiguration();
    yawCanCoderConfig.MagnetSensor.MagnetOffset = YAW_MAGNETIC_OFFSET.in(Rotations);
    yawCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    yawEncoder.getConfigurator().apply(yawCanCoderConfig);

    // Configure the yaw motor
    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    yawTalonConfig.MotorOutput.NeutralMode = Brake;
    yawTalonConfig.Feedback.RotorToSensorRatio = YAW_ROTOR_TO_SENSOR_RATIO;
    yawTalonConfig.Feedback.FeedbackRemoteSensorID = yawEncoder.getDeviceID();
    yawTalonConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
    yawTalonConfig.Slot0 = Slot0Configs.from(YAW_SLOT_CONFIGS);
    yawTalonConfig.MotionMagic = YAW_MOTION_MAGIC_CONFIGS;
    yawTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    yawTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = YAW_LIMIT_FORWARD.in(Rotations);
    yawTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    yawTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = YAW_LIMIT_REVERSE.in(Rotations);
    yawMotor.getConfigurator().apply(yawTalonConfig);

    yawPositionSignal = yawMotor.getPosition();

    // Configure the pitch CANCoder
    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.MagnetOffset = PITCH_MAGNETIC_OFFSET.in(Rotations);
    pitchCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    // Configure the pitch motor
    var pitchTalonConfig = new TalonFXConfiguration();
    pitchTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pitchTalonConfig.MotorOutput.NeutralMode = Brake;
    pitchTalonConfig.Feedback.RotorToSensorRatio = PITCH_ROTOR_TO_SENSOR_RATIO;
    pitchTalonConfig.Feedback.FeedbackRemoteSensorID = pitchEncoder.getDeviceID();
    pitchTalonConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
    pitchTalonConfig.Slot0 = Slot0Configs.from(PITCH_SLOT_CONFIGS);
    pitchTalonConfig.MotionMagic = PITCH_MOTION_MAGIC_CONFIGS;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PITCH_LIMIT_FORWARD.in(Rotations);
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PITCH_LIMIT_REVERSE.in(Rotations);
    pitchMotor.getConfigurator().apply(pitchTalonConfig);

    pitchPositionSignal = pitchMotor.getPosition();

    // Configure the roller motor
    var rollerTalonConfig = new TalonFXConfiguration();
    rollerTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerTalonConfig.MotorOutput.NeutralMode = Brake;
    rollerTalonConfig.Slot0 = Slot0Configs.from(ROLLER_VELOCITY_SLOT_CONFIGS);
    rollerMotor.getConfigurator().apply(rollerTalonConfig);
  }

  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine.dynamic(direction).withName("SysId turret yaw dynam " + direction)
        .finallyDo(this::stopYaw);
  }

  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine.quasistatic(direction).withName("SysId turret yaw quasi " + direction)
        .finallyDo(this::stopYaw);
  }

  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine.dynamic(direction).withName("SysId turret pitch dynam " + direction)
        .finallyDo(this::stopPitch);
  }

  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine.quasistatic(direction).withName("SysId turret pitch quasi " + direction)
        .finallyDo(this::stopPitch);
  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId turret rollers dynam " + direction)
        .finallyDo(this::stopRollers);
  }

  public Command sysIdRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId turret rollers quasi " + direction)
        .finallyDo(this::stopRollers);
  }

  /**
   * Sets the turret yaw (rotation around the Z axis) position target. Zero is robot foward, using the WPILib unit
   * circle.
   * @param yaw robot relative yaw
   */
  public void setYawTarget(Measure<Angle> yaw) {
    yawMotor.setControl(yawControl.withPosition(translateYaw(yaw)));
  }

  /**
   * Translates a yaw value from robot yaw to turret yaw, or vice versa. This is needed because turret 0 is facing robot
   * backward.
   * @param yaw robot centric yaw to convert to turret yaw, or turret yaw to translate to robot yaw
   * @return translated yaw in rotations
   */
  private double translateYaw(Measure<Angle> yaw) {
    return Math.IEEEremainder(yaw.in(Rotations) + 0.5, 1);
  }

  /**
   * Sets the turret pitch (rotation around the Y axis) position target. Zero is horizontal, upward is positive
   * (since the turret is facing backward).
   * @param pitch pitch
   */
  public void setPitchTarget(Measure<Angle> pitch) {
    pitchMotor.setControl(pitchControl.withPosition(pitch.in(Rotations)));
  }

  /**
   * Loads a note from the amper/intake. Must likely {@link #prepareToExchange()} and {@link #isAtYawAndPitchTarget()}
   * should be called first.
   */
  public void load() {
    runRollers(LOAD_VELOCITY);
    setYawTarget(INTAKE_YAW_POSITION);
    setPitchTarget(INTAKE_PITCH_POSITION);
  }

  /**
   * Ejects a note into the amper/intake
   */
  public void eject() {
    runRollers(EJECT_VELOCITY);
  }

  /**
   * Unloads a note into the shooter
   */
  public void shoot() {
    runRollers(SHOOT_VELOCITY);
  }

  /**
   * Runs the rollers
   * @param velocity velocity for rollers
   */
  public void runRollers(Measure<Velocity<Angle>> velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  /**
   * Stops the roller motor
   */
  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  /**
   * Stops the yaw motor
   */
  public void stopYaw() {
    yawMotor.stopMotor();
  }

  /**
   * Stops the pitch motor
   */
  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  /**
   * Stops all of the motors
   */
  public void stop() {
    stopYaw();
    stopPitch();
    stopRollers();
  }

  /**
   * Gets the turret yaw and pitch into position to load / eject. Check {@link #isAtYawAndPitchTarget()} to see find out
   * when the turret is ready
   */
  public void prepareToExchange() {
    setYawTarget(INTAKE_YAW_POSITION);
    setPitchTarget(INTAKE_PITCH_POSITION);
  }

  /**
   * Indicates if the yaw and pitch are at their target positions
   * @return true if the yaw and pitch are at their target positions
   */
  public boolean isAtYawAndPitchTarget() {
    BaseStatusSignal.refreshAll(yawPositionSignal, pitchPositionSignal);
    return Math.abs(yawControl.Position - yawPositionSignal.getValueAsDouble()) < YAW_TOLERANCE.in(Rotations)
        && Math.abs(pitchControl.Position - pitchPositionSignal.getValueAsDouble()) < PITCH_TOLERANCE.in(Rotations);
  }

}
