package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static com.revrobotics.CANSparkBase.ControlType.kPosition;
import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kReverse;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import static com.revrobotics.SparkPIDController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.AIM_FORWARD_LIMIT;
import static frc.robot.Constants.ShooterConstants.AIM_GRAVITY_FF;
import static frc.robot.Constants.ShooterConstants.AIM_OFFSET;
import static frc.robot.Constants.ShooterConstants.AIM_REVERSE_LIMIT;
import static frc.robot.Constants.ShooterConstants.AIM_kD;
import static frc.robot.Constants.ShooterConstants.AIM_kI;
import static frc.robot.Constants.ShooterConstants.AIM_kP;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_AIM;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_POSITION_SLOT_CONFIG;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeftMotor = new TalonFX(DEVICE_ID_LEFT);
  private final TalonFX shooterRightMotor = new TalonFX(DEVICE_ID_RIGHT);

  private final CANSparkMax aimMotor = new CANSparkMax(DEVICE_ID_AIM, kBrushless);
  private final SparkPIDController aimPidController = aimMotor.getPIDController();
  private final AbsoluteEncoder aimEncoder;

  private final VelocityTorqueCurrentFOC shooterVelocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC shooterPositionControl = new PositionTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // SysId routines  
  private final SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterLeftMotor.setControl(voltageControl.withOutput(volts.in(Volts)));
        shooterRightMotor.setControl(voltageControl.withOutput(volts.in(Volts)));
      }, null, this));

  private final SysIdRoutine aimSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(3.0), null, null),
      new SysIdRoutine.Mechanism((volts) -> aimMotor.setVoltage(volts.in(Volts)), null, this));
  
  private double aimTargetRotations = 0.0;
  private boolean holdAimPosition = false;

  public ShooterSubsystem() {
    // Configure shooter motors
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG);
    shooterMotorConfig.Slot1 = Slot1Configs.from(SHOOTER_POSITION_SLOT_CONFIG);
    shooterMotorConfig.MotorOutput.NeutralMode = Coast;
    shooterMotorConfig.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECHANISM_RATIO;
    shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);

    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterRightMotor.getConfigurator().apply(shooterMotorConfig);

    // Configure aim motor
    aimMotor.restoreFactoryDefaults();
    aimMotor.enableVoltageCompensation(12.0);
    aimMotor.setSoftLimit(kForward, AIM_FORWARD_LIMIT);
    aimMotor.setSoftLimit(kReverse, AIM_REVERSE_LIMIT);
    aimMotor.enableSoftLimit(kForward, true);
    aimMotor.enableSoftLimit(kReverse, true);
    aimMotor.setIdleMode(kBrake);
    aimMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    aimMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);

    aimPidController.setP(AIM_kP);
    aimPidController.setI(AIM_kI);
    aimPidController.setD(AIM_kD);
    aimPidController.setPositionPIDWrappingMinInput(0);
    aimPidController.setPositionPIDWrappingMaxInput(1);
    aimPidController.setPositionPIDWrappingEnabled(true);
    
    // Configure aim encoder
    aimEncoder = aimMotor.getAbsoluteEncoder(kDutyCycle);
    aimEncoder.setAverageDepth(64);
    aimEncoder.setZeroOffset(AIM_OFFSET.in(Rotations));
    aimPidController.setFeedbackDevice(aimEncoder);

    aimMotor.burnFlash();
  }

  /**
   * Returns a new command that active stops the shooter to hold the note
   * @return new command
   */
  public Command activeStopShooterCommand() {
    return runOnce(this::activeStopShooter).finallyDo(this::stopShooter);
  }

  /**
   * Returns a new command to spin the shooter at a velocity
   * @param velocity velocity to spin the shooter
   * @return new command
   */
  public Command spinShooterCommand(Measure<Velocity<Angle>> velocity) {
    return run(() -> spinShooterWheels(velocity)).finallyDo(this::stopShooter);
  }

  public Command spinShooterAndAimCommand(Measure<Velocity<Angle>> velocity, Measure<Angle> aimAngle) {
    return run(() -> {
      spinShooterWheels(velocity);
      setAimPosition(aimAngle);
    }).finallyDo(this::stopShooter);
  }

  /**
   * Returns a new command to rotate the shooter a given distance
   * @param distance distance to rotate, relative to the current position
   * @return new command
   */
  public Command rotateShooterCommand(Measure<Angle> distance) {
    return run(() -> rotateShooterWheels(distance)).finallyDo(this::stopShooter);
  }

  /**
   * Returns a new command to set the shooter's aim angle
   * @param angle angle of shooter
   * @return new command
   */
  public Command setAimAngleCommand(Measure<Angle> angle) {
    return runOnce(() -> setAimPosition(angle));
  }

  public Command sysIdShooterDynamicCommand(Direction direction) {
    return shooterSysIdRoutine.dynamic(direction).withName("Shooter dynam " + direction)
        .finallyDo(this::stopShooter);
  }

  public Command sysIdShooterQuasistaticCommand(Direction direction) {
    return shooterSysIdRoutine.quasistatic(direction).withName("Shooter quasi " + direction)
        .finallyDo(this::stopShooter);
  }

  public Command sysIdAimDynamicCommand(Direction direction) {
    return runOnce(() -> holdAimPosition = false)
        .andThen(aimSysIdRoutine.dynamic(direction)).withName("Aim dynam " + direction)
        .finallyDo(this::stopAim);
  }

  public Command sysIdAimQuasistaticCommand(Direction direction) {
    return runOnce(() -> holdAimPosition = false)
        .andThen(aimSysIdRoutine.quasistatic(direction)).withName("Aim quasi " + direction)
        .finallyDo(this::stopAim);
  }

  @Override
  public void periodic() {
    if (holdAimPosition) {
      // Need to periodically set the reference because gravity FF will change as the shooter moves
      var cosineScalar = Units.rotationsToRadians(aimEncoder.getPosition());
      var gravityFF = AIM_GRAVITY_FF.in(Volts) * cosineScalar;

      aimPidController.setReference(aimTargetRotations, kPosition, 0, gravityFF, kVoltage);
    }
  }

  private void spinShooterWheels(Measure<Velocity<Angle>> velocity) {
    shooterLeftMotor.setControl(shooterVelocityControl.withVelocity(velocity.in(RotationsPerSecond)));
    shooterRightMotor.setControl(shooterVelocityControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  private void rotateShooterWheels(Measure<Angle> distance) {
    shooterLeftMotor.setPosition(0.0);
    shooterLeftMotor.setControl(shooterPositionControl.withPosition(distance.in(Rotations)));
    shooterRightMotor.setPosition(0.0);
    shooterRightMotor.setControl(shooterPositionControl.withPosition(distance.in(Rotations)));
  }

  private void activeStopShooter() {
    shooterLeftMotor.setPosition(0.0);
    shooterLeftMotor.setControl(shooterPositionControl.withPosition(0.0));
    shooterRightMotor.setPosition(0.0);
    shooterRightMotor.setControl(shooterPositionControl.withPosition(0.0));
  }

  private void setAimPosition(Measure<Angle> angle) {
    holdAimPosition = true;
    aimTargetRotations = angle.in(Rotations);
  }

  private void stopShooter() {
    shooterRightMotor.stopMotor();
    shooterLeftMotor.stopMotor();
  }

  private void stopAim() {
    aimMotor.stopMotor();
  }

}