package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static com.revrobotics.CANSparkBase.ControlType.kPosition;
import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.ACTUATOR_ERROR_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.AIM_GRAVITY_FF;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_ACTUATOR_MOTOR;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEFT;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_RIGHT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG_TOP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_TOLERANCE;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeftMotor = new TalonFX(DEVICE_ID_SHOOTER_LEFT, CANIVORE_BUS_NAME);
  private final TalonFX shooterRightMotor = new TalonFX(DEVICE_ID_SHOOTER_RIGHT, CANIVORE_BUS_NAME);
  private final CANSparkMax actuatorMotor = new CANSparkMax(DEVICE_ID_ACTUATOR_MOTOR, MotorType.kBrushless);
  // creates new devices and sets them to the device Ids set in
  // Constants.java

  private static final double ENCODER_OFFSET = -0.2285f;
  private static final float LIMIT_BOTTOM = 0.12f;
  private static final float LIMIT_TOP = 0.4272f;
  private static final double LIMIT_TOP_RADIANS = Units.rotationsToRadians(LIMIT_TOP + ENCODER_OFFSET);
  private static final double LIMIT_BOTTOM_RADIANS = Units.rotationsToRadians(LIMIT_BOTTOM + ENCODER_OFFSET);
  private boolean isActiveStopped = false;
  public boolean hasRing = false;
  private SparkAbsoluteEncoder actuatorEncoder;
  private final SparkPIDController pidController;
  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private double aimTargetRotations = 0.0;
  private boolean holdAimPosition = false;


  private SysIdRoutine shooterMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterRightMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
        shooterLeftMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private final VelocityTorqueCurrentFOC shooterMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false).withSlot(0);

  public ShooterSubsystem() {
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_TOP);
    shooterMotorConfig.MotorOutput.NeutralMode = Coast;
    shooterMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 1;
    shooterMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM);
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterRightMotor.getConfigurator().apply(shooterMotorConfig);

    // Get the through-bore-encoder absolute encoder
    actuatorEncoder = actuatorMotor.getAbsoluteEncoder(kDutyCycle);
    actuatorEncoder.setAverageDepth(64);
    pidController = actuatorMotorConfig(actuatorMotor, true);
    pidController.setFeedbackDevice(actuatorEncoder);
  }

  private static SparkPIDController actuatorMotorConfig(CANSparkMax sparkMax, boolean invert) {
    SparkPIDController pidController = sparkMax.getPIDController();
    sparkMax.restoreFactoryDefaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setOpenLoopRampRate(0.1);
    sparkMax.setClosedLoopRampRate(0.1);
    sparkMax.setInverted(invert);
    pidController.setP(IndexerConstants.BELT_kP);
    pidController.setFF(0.00009);
    sparkMax.getEncoder();
    sparkMax.burnFlash();
    sparkMax.setIdleMode(IdleMode.kCoast);
    return pidController;

  }

  public void spinShooterWheel(double shooterRPS) {
    shooterLeftMotor.setControl(shooterMotorVelocity.withVelocity(shooterRPS));
    shooterRightMotor.setControl(shooterMotorVelocity.withVelocity(shooterRPS));
  }

  public Command sysIdShooterMotorQuasiCommand(Direction direction) {
    return shooterMotorSysIdRoutine.quasistatic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterMotorDynamCommand(Direction direction) {
    return shooterMotorSysIdRoutine.dynamic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  @Override
  public void periodic() {
    if (holdAimPosition) {
      // Need to periodically set the reference because gravity FF will change as the
      // shooter moves
      var cosineScalar = Math.cos(rotationsToRadians(actuatorEncoder.getPosition()));
      var gravityFF = AIM_GRAVITY_FF.in(Volts) * cosineScalar;

      pidController.setReference(aimTargetRotations, kPosition); //ask andy today
    }
  }

  public void setAimPosition(Measure<Angle> angle) {
    holdAimPosition = true;
    aimTargetRotations = angle.in(Rotations);
  }

  public void activeStop() {
    if (!isActiveStopped) {
      shooterLeftMotor.setControl(shooterMotorVelocity.withVelocity(0));
      shooterRightMotor.setControl(shooterMotorVelocity.withVelocity(0));
    }
    isActiveStopped = true;
  }

  /**
   * Gets the wrist position Zero is horizontal, up is positive
   * 
   * @return position in radians
   */
  public double getWristPosition() {
    return Units.rotationsToRadians(actuatorEncoder.getPosition() + ENCODER_OFFSET);
  }

  public void actuatorUp(Measure<Voltage> volts) {
    actuatorMotorConfig(actuatorMotor, true);
    actuatorMotor.setVoltage(volts.in(Volts));
  }

  public void actuatorDown(Measure<Voltage> volts) {
    actuatorMotorConfig(actuatorMotor, false);
    actuatorMotor.setVoltage(volts.in(Volts));
  }

  /**
   * Gets the wrist velocity in radians per second
   * 
   * @return wrist velocity in radians per second
   */
  public double getWristVelocity() {
    return Units.rotationsToDegrees(actuatorEncoder.getVelocity());
  }

  public double getShooterLeftVelocity() {
    return shooterLeftMotor.getVelocity().getValueAsDouble();
  }

  public double getShooterRightVelocity() {
    return shooterRightMotor.getVelocity().getValueAsDouble();
  }

  private boolean isAimAtTargetPosition() {
    return Math.abs(actuatorEncoder.getPosition() - aimTargetRotations) < ACTUATOR_ERROR_TOLERANCE.in(Rotations);
  }

  private boolean checkShooterSpeed(double shooterSpeedGoal) {
    return ((Math.abs(getShooterLeftVelocity() - shooterSpeedGoal) <= SHOOTER_VELOCITY_TOLERANCE) &&
        (Math.abs(getShooterRightVelocity() - shooterSpeedGoal) <= SHOOTER_VELOCITY_TOLERANCE));
  }

  public boolean isShooterReady() {
    return(checkShooterSpeed(40) && isAimAtTargetPosition());
  }

  public void shootDutyCycle(double speed) {
    shooterRightMotor.set(speed);
    shooterLeftMotor.set(speed);
    isActiveStopped = false;
  }

  public void stop() {
    shooterRightMotor.stopMotor();
    shooterLeftMotor.stopMotor();
    isActiveStopped = false;

  }

  public void actuatorRotate(double radians) {
    var target = MathUtil.clamp(radians, LIMIT_BOTTOM_RADIANS, LIMIT_TOP_RADIANS);
    new TrapezoidProfile.State(target, 0.0);
  }

  public void actuatorStop() {
    actuatorMotor.setVoltage(0);
  }
}