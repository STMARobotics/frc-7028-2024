package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.FeedbackSensorSourceValue.FusedCANcoder;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.TurretConstants.AMP_PITCH;
import static frc.robot.Constants.TurretConstants.AMP_YAW;
import static frc.robot.Constants.TurretConstants.BABY_BIRD_PITCH;
import static frc.robot.Constants.TurretConstants.BABY_BIRD_ROLLER_VELOCITY;
import static frc.robot.Constants.TurretConstants.BABY_BIRD_YAW;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_NOTE_SENSOR;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_PITCH_ENCODER;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_PITCH_MOTOR;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_ROLLER_MOTOR;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_YAW_ENCODER;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_YAW_MOTOR;
import static frc.robot.Constants.TurretConstants.EJECT_VELOCITY;
import static frc.robot.Constants.TurretConstants.INTAKE_PITCH;
import static frc.robot.Constants.TurretConstants.INTAKE_PITCH_TOLERANCE;
import static frc.robot.Constants.TurretConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.TurretConstants.INTAKE_YAW;
import static frc.robot.Constants.TurretConstants.INTAKE_YAW_TOLERANCE;
import static frc.robot.Constants.TurretConstants.NOTE_SENSOR_DISTANCE_THRESHOLD;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.PITCH_MAGNETIC_OFFSET;
import static frc.robot.Constants.TurretConstants.PITCH_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.PITCH_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.PITCH_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.PITCH_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.PITCH_TOLERANCE;
import static frc.robot.Constants.TurretConstants.ROBOT_TO_TURRET;
import static frc.robot.Constants.TurretConstants.ROLLER_PEAK_FORWARD_CURRENT;
import static frc.robot.Constants.TurretConstants.ROLLER_PEAK_REVERSE_CURRENT;
import static frc.robot.Constants.TurretConstants.ROLLER_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.SHOOTING_YAW_CORRECTION;
import static frc.robot.Constants.TurretConstants.SHOOT_VELOCITY;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_MAGNETIC_OFFSET;
import static frc.robot.Constants.TurretConstants.YAW_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.TurretConstants.YAW_SHOOT_LIMIT_FORWARD;
import static frc.robot.Constants.TurretConstants.YAW_SHOOT_LIMIT_REVERSE;
import static frc.robot.Constants.TurretConstants.YAW_SLOT_CONFIGS;
import static frc.robot.Constants.TurretConstants.YAW_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.TurretConstants.YAW_TOLERANCE;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/**
 * Subsystem for the turret. The turret has yaw and pitch control, and it has a set of rollers that
 * hold a note (a.k.a. indexer)
 */
public class TurretSubsystem extends SubsystemBase {
  private final TalonFX yawMotor = new TalonFX(DEVICE_ID_YAW_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder yawEncoder = new CANcoder(DEVICE_ID_YAW_ENCODER, CANIVORE_BUS_NAME);

  private final TalonFX pitchMotor = new TalonFX(DEVICE_ID_PITCH_MOTOR, CANIVORE_BUS_NAME);
  private final CANcoder pitchEncoder = new CANcoder(DEVICE_ID_PITCH_ENCODER, CANIVORE_BUS_NAME);

  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLER_MOTOR, CANIVORE_BUS_NAME);

  private final LaserCan noteSensor = new LaserCan(DEVICE_ID_NOTE_SENSOR);

  private final MotionMagicVoltage yawControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage pitchControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC rollerControl =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);

  private final TorqueCurrentFOC rollerSysIdControl = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Double> yawPosition;
  private final StatusSignal<Double> yawVelocity;
  private final StatusSignal<Double> pitchPosition;
  private final StatusSignal<Double> pitchVelocity;

  // Reusable objects to reduce memory pressure from reallocation
  private final MutableMeasure<Angle> yawAngle = MutableMeasure.zero(Rotations);
  private final MutableMeasure<Angle> pitchAngle = MutableMeasure.zero(Rotations);

  // SysId routines
  private final SysIdRoutine yawSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, Volts.of(4.0), null, SysIdRoutineSignalLogger.logState()),
          new SysIdRoutine.Mechanism(
              (volts) -> yawMotor.setControl(voltageControl.withOutput(volts.in(Volts))),
              null,
              this));

  private final SysIdRoutine pitchSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Seconds), Volts.of(2.0), null, SysIdRoutineSignalLogger.logState()),
          new SysIdRoutine.Mechanism(
              (volts) -> pitchMotor.setControl(voltageControl.withOutput(volts.in(Volts))),
              null,
              this));

  // NOTE: the output type for the rollers is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Seconds.of(1)),
              Volts.of(30),
              null,
              SysIdRoutineSignalLogger.logState()),
          new SysIdRoutine.Mechanism(
              (amps) -> rollerMotor.setControl(rollerSysIdControl.withOutput(amps.in(Volts))),
              null,
              this));

  public TurretSubsystem() {
    // Configure the yaw CANCoder
    var yawCanCoderConfig = new CANcoderConfiguration();
    yawCanCoderConfig.MagnetSensor.MagnetOffset = YAW_MAGNETIC_OFFSET.in(Rotations);
    yawCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    yawEncoder.getConfigurator().apply(yawCanCoderConfig);

    // Configure the yaw motor
    var yawTalonConfig = new TalonFXConfiguration();
    yawTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    yawTalonConfig.MotorOutput.NeutralMode = Brake;
    yawTalonConfig.CurrentLimits.StatorCurrentLimit = YAW_STATOR_CURRENT_LIMIT.in(Amps);
    yawTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    yawTalonConfig.CurrentLimits.SupplyCurrentLimit = YAW_SUPPLY_CURRENT_LIMIT.in(Amps);
    yawTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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

    yawPosition = yawMotor.getPosition();
    yawVelocity = yawMotor.getVelocity();

    // Configure the pitch CANCoder
    var pitchCanCoderConfig = new CANcoderConfiguration();
    pitchCanCoderConfig.MagnetSensor.MagnetOffset = PITCH_MAGNETIC_OFFSET.in(Rotations);
    pitchCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pitchEncoder.getConfigurator().apply(pitchCanCoderConfig);

    // Configure the pitch motor
    var pitchTalonConfig = new TalonFXConfiguration();
    pitchTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pitchTalonConfig.MotorOutput.NeutralMode = Brake;
    pitchTalonConfig.CurrentLimits.StatorCurrentLimit = PITCH_STATOR_CURRENT_LIMIT.in(Amps);
    pitchTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pitchTalonConfig.CurrentLimits.SupplyCurrentLimit = PITCH_SUPPLY_CURRENT_LIMIT.in(Amps);
    pitchTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pitchTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
    pitchTalonConfig.Feedback.RotorToSensorRatio = PITCH_ROTOR_TO_SENSOR_RATIO;
    pitchTalonConfig.Feedback.FeedbackRemoteSensorID = pitchEncoder.getDeviceID();
    pitchTalonConfig.Feedback.FeedbackSensorSource = FusedCANcoder;
    pitchTalonConfig.Slot0 = Slot0Configs.from(PITCH_SLOT_CONFIGS);
    pitchTalonConfig.MotionMagic = PITCH_MOTION_MAGIC_CONFIGS;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PITCH_LIMIT_FORWARD.in(Rotations);
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pitchTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PITCH_LIMIT_REVERSE.in(Rotations);
    pitchMotor.getConfigurator().apply(pitchTalonConfig);

    pitchPosition = pitchMotor.getPosition();
    pitchVelocity = pitchMotor.getVelocity();

    // Configure the roller motor
    var rollerTalonConfig = new TalonFXConfiguration();
    rollerTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerTalonConfig.MotorOutput.NeutralMode = Brake;
    rollerTalonConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);
    rollerTalonConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_CURRENT_LIMIT.in(Amps);
    rollerTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = ROLLER_PEAK_FORWARD_CURRENT.in(Amps);
    rollerTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = ROLLER_PEAK_REVERSE_CURRENT.in(Amps);
    rollerMotor.getConfigurator().apply(rollerTalonConfig);

    // Configure the note sensor
    try {
      noteSensor.setRangingMode(RangingMode.SHORT);
      noteSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      DriverStation.reportError("Failed to confgure turret LaserCAN: " + e.getMessage(), false);
    }
  }

  public Command sysIdYawDynamicCommand(Direction direction) {
    return yawSysIdRoutine
        .dynamic(direction)
        .withName("SysId turret yaw dynam " + direction)
        .finallyDo(this::stopYaw);
  }

  public Command sysIdYawQuasistaticCommand(Direction direction) {
    return yawSysIdRoutine
        .quasistatic(direction)
        .withName("SysId turret yaw quasi " + direction)
        .finallyDo(this::stopYaw);
  }

  public Command sysIdPitchDynamicCommand(Direction direction) {
    return pitchSysIdRoutine
        .dynamic(direction)
        .withName("SysId turret pitch dynam " + direction)
        .finallyDo(this::stopPitch);
  }

  public Command sysIdPitchQuasistaticCommand(Direction direction) {
    return pitchSysIdRoutine
        .quasistatic(direction)
        .withName("SysId turret pitch quasi " + direction)
        .finallyDo(this::stopPitch);
  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine
        .dynamic(direction)
        .withName("SysId turret rollers dynam " + direction)
        .finallyDo(this::stopRollers);
  }

  public Command sysIdRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine
        .quasistatic(direction)
        .withName("SysId turret rollers quasi " + direction)
        .finallyDo(this::stopRollers);
  }

  /**
   * Checks in the turret has a note loaded
   *
   * @return true if the turret has a note, otherwise false
   */
  public boolean hasNote() {
    var measure = noteSensor.getMeasurement();
    if (measure == null || measure.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return false;
    }
    return measure.distance_mm < NOTE_SENSOR_DISTANCE_THRESHOLD.in(Millimeters);
  }

  /**
   * Sets the turret pitch (rotation around the Y axis) position target. Zero is horizontal, upward
   * is positive (since the turret is facing backward).
   *
   * @param pitch pitch
   */
  public void moveToPitchPosition(Measure<Angle> pitch) {
    pitchMotor.setControl(pitchControl.withPosition(pitch.in(Rotations)));
  }

  /**
   * Loads a note from the intake. Must likely {@link #prepareToIntake()} and {@link
   * #isAtYawAndPitchTarget()} should be called first.
   */
  public void intake() {
    runRollers(INTAKE_VELOCITY);
    moveToYawPosition(INTAKE_YAW);
    moveToPitchPosition(INTAKE_PITCH);
  }

  /**
   * Ejects a note into the intake. Must likely {@link #prepareToIntake()} and {@link
   * #isAtYawAndPitchTarget()} should be called first.
   */
  public void eject() {
    runRollers(EJECT_VELOCITY);
    moveToYawPosition(INTAKE_YAW);
    moveToPitchPosition(INTAKE_PITCH);
  }

  /** Unloads a note into the shooter */
  public void shoot() {
    runRollers(SHOOT_VELOCITY);
  }

  /**
   * Runs the rollers
   *
   * @param velocity velocity for rollers
   */
  public void runRollers(Measure<Velocity<Angle>> velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  /**
   * Gets the turret yaw and pitch into position to intake / eject. Check {@link
   * #isInIntakePosition()} to find out when the turret is ready
   */
  public void prepareToIntake() {
    moveToYawPosition(INTAKE_YAW);
    moveToPitchPosition(INTAKE_PITCH);
    stopRollers();
  }

  /**
   * Gets the turret yaw and pitch in position to amp. Check {@link #isAtYawAndPitchTarget()} to
   * find out when the turret is ready
   */
  public void prepareToAmp() {
    moveToYawPosition(AMP_YAW);
    moveToPitchPosition(AMP_PITCH);
    stopRollers();
  }

  /**
   * Gets the turret yaw and pitch in position for baby bird. Check {@link #isAtYawAndPitchTarget()}
   * to find out when the turret is ready
   */
  public void prepareToBabyBird() {
    moveToYawPosition(BABY_BIRD_YAW);
    moveToPitchPosition(BABY_BIRD_PITCH);
    runRollers(BABY_BIRD_ROLLER_VELOCITY);
  }

  /**
   * Indicates if the yaw and pitch are at their target positions
   *
   * @return true if the yaw and pitch are at their target positions
   */
  public boolean isAtYawAndPitchTarget() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity, pitchPosition, pitchVelocity);
    return isInTolerance(yawControl.Position, yawPosition, yawVelocity, YAW_TOLERANCE.in(Rotations))
        && isInTolerance(
            pitchControl.Position, pitchPosition, pitchVelocity, PITCH_TOLERANCE.in(Rotations));
  }

  /**
   * Indicates if the turret is in the intake position. This is similar to {@link
   * #isAtYawAndPitchTarget()}, except the tolerance is wider.
   *
   * @return true if the yaw and pitch are at the intake position
   */
  public boolean isInIntakePosition() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity, pitchPosition, pitchVelocity);
    return isInTolerance(
            yawControl.Position, yawPosition, yawVelocity, INTAKE_YAW_TOLERANCE.in(Rotations))
        && isInTolerance(
            pitchControl.Position,
            pitchPosition,
            pitchVelocity,
            INTAKE_PITCH_TOLERANCE.in(Rotations));
  }

  /**
   * Indicates if the yaw is at its target position
   *
   * @return true if the yaw is at the target position
   */
  public boolean isAtYawTarget() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
    return isInTolerance(
        yawControl.Position, yawPosition, yawVelocity, YAW_TOLERANCE.in(Rotations));
  }

  /**
   * Indicates if the pitch is at its target position
   *
   * @return true if the pitch is at the target position
   */
  public boolean isAtPitchTarget() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    return isInTolerance(
        pitchControl.Position, pitchPosition, pitchVelocity, PITCH_TOLERANCE.in(Rotations));
  }

  /**
   * Sets the turret yaw (rotation around the Z axis) position target. Zero is robot forward, using
   * the WPILib unit circle.
   *
   * @param yaw robot relative yaw
   */
  public void moveToYawPosition(Measure<Angle> yaw) {
    yawMotor.setControl(yawControl.withPosition(translateYaw(yaw)));
  }

  /**
   * Sets the turret yaw (rotation around the z axix) position target, but for shooting. The
   * difference between this and {@link #moveToYawPosition(Measure)} is that this method only turns
   * the turret within the bounds where shooting is possible, and it applies correct for the shooter
   * not launching perfectly straight. Zero is robot forward, using the WPILib unit circle.
   *
   * @param yaw robot relative yaw
   */
  public void moveToShootingYawPosition(Measure<Angle> yaw) {
    // Clamp to shooting range, and correct for the fact that the shooter doesn't shoot straight
    var targetYaw =
        MathUtil.clamp(
            translateYaw(yaw.plus(SHOOTING_YAW_CORRECTION)),
            YAW_SHOOT_LIMIT_REVERSE.in(Rotations),
            YAW_SHOOT_LIMIT_FORWARD.in(Rotations));

    yawMotor.setControl(yawControl.withPosition(targetYaw));
  }

  /**
   * Gets the current pitch angle
   *
   * @return pitch
   */
  public Measure<Angle> getPitch() {
    BaseStatusSignal.refreshAll(pitchPosition, pitchVelocity);
    var compensatedPitch =
        BaseStatusSignal.getLatencyCompensatedValue(pitchPosition, pitchVelocity);
    return pitchAngle.mut_replace(compensatedPitch, Rotations);
  }

  /**
   * Gets the current yaw angle
   *
   * @return yaw
   */
  public Measure<Angle> getYaw() {
    BaseStatusSignal.refreshAll(yawPosition, yawVelocity);
    var compensatedYaw = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    var translatedRotations = translateYaw(yawAngle.mut_replace(compensatedYaw, Rotations));
    return yawAngle.mut_replace(translatedRotations, Rotations);
  }

  /** Stops the roller motor */
  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  /** Stops the yaw motor */
  public void stopYaw() {
    yawMotor.stopMotor();
  }

  /** Stops the pitch motor */
  public void stopPitch() {
    pitchMotor.stopMotor();
  }

  /** Stops all of the motors */
  public void stop() {
    stopYaw();
    stopPitch();
    stopRollers();
  }

  /**
   * Checks if the specified yaw angle is within the range from where the turret can shoot a note
   *
   * @param angle angle to check
   * @return true if in range, false if out of range
   */
  public static boolean isYawInShootingRange(Measure<Angle> angle) {
    var angleRotations = translateYaw(angle);
    return angleRotations < YAW_SHOOT_LIMIT_FORWARD.in(Rotations)
        && angleRotations > YAW_SHOOT_LIMIT_REVERSE.in(Rotations);
  }

  public static Translation2d getTurretTranslation(Pose2d robotPose) {
    return robotPose.getTranslation().plus(ROBOT_TO_TURRET.rotateBy(robotPose.getRotation()));
  }

  /**
   * Checks if a signal is within tolerance of a target
   *
   * @param target target
   * @param value status signal for current value. You may want to refresh first, this method does
   *     not refresh
   * @param slope derivative of the signal that defines the slope for latency compensation, this
   *     method does not refresh
   * @param tolerance tolerance
   * @return true if value is within tolerance of the target
   */
  private static boolean isInTolerance(
      double target, StatusSignal<Double> value, StatusSignal<Double> slope, double tolerance) {
    var compensated = BaseStatusSignal.getLatencyCompensatedValue(value, slope);
    return Math.abs(target - compensated) < tolerance;
  }

  /**
   * Translates a yaw value from robot yaw to turret yaw, or vice versa. This is needed because
   * turret 0 is facing robot backward.
   *
   * @param yaw robot centric yaw to convert to turret yaw, or turret yaw to translate to robot yaw
   * @return translated yaw in rotations
   */
  private static double translateYaw(Measure<Angle> yaw) {
    return Math.IEEEremainder(yaw.in(Rotations) + 0.5, 1);
  }
}
