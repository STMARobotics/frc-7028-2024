package frc.robot.subsystems;

import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_ACTUATOR_MOTOR;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEFT;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_RIGHT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_OFFSET;
import static frc.robot.Constants.ShooterConstants.WRIST_POSITION_OFFSET;

import java.util.Objects;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;


public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeftMotor = new TalonFX(DEVICE_ID_SHOOTER_LEFT);
  private final TalonFX shooterRightMotor = new TalonFX(DEVICE_ID_SHOOTER_RIGHT);
  private final CANSparkMax actuatorMotor = new CANSparkMax(DEVICE_ID_ACTUATOR_MOTOR, MotorType.kBrushless);
  // creates new devices and sets them to the device Ids set in
  // Constants.java

  private TrapezoidProfile.State goal = null;
  private static final double ENCODER_OFFSET = -0.2285f;
  private static final float LIMIT_BOTTOM = 0.12f;
  private static final float LIMIT_TOP = 0.4272f;
  private static final double LIMIT_TOP_RADIANS = Units.rotationsToRadians(LIMIT_TOP + ENCODER_OFFSET);
  private static final double LIMIT_BOTTOM_RADIANS = Units.rotationsToRadians(LIMIT_BOTTOM + ENCODER_OFFSET);
  private static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(40, 11);
  private boolean isActiveStopped = false;
  public boolean hasRing = false;
  private SparkAbsoluteEncoder actuatorEncoder;
  private final SparkPIDController pidController;

  // Offset in rotations to add to encoder value - offset from arm horizontal to
  // sensor zero

  private VoltageOut voltageRequest = new VoltageOut(0);

  private SysIdRoutine ShooterMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterRightMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
        shooterLeftMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private final VelocityTorqueCurrentFOC shooterMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);

  public ShooterSubsystem() {
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
    shooterMotorConfig.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    shooterMotorConfig.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Get the through-bore-encoder absolute encoder
    actuatorEncoder = actuatorMotor.getAbsoluteEncoder(kDutyCycle);
    actuatorEncoder.setInverted(true);
    actuatorEncoder.setAverageDepth(64);
    pidController = actuatorMotor.getPIDController();
    pidController.setFeedbackDevice(actuatorEncoder);

    double kP = 3.9;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = .4;
    double kMinOutput = -.3;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    pidController.setPositionPIDWrappingMaxInput(1);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingEnabled(true);

    shooterLeftMotor.setInverted(true);

    shooterRightMotor.setNeutralMode(NeutralModeValue.Brake);
    shooterLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
    shooterRightMotor.getConfigurator().apply(shooterMotorConfig);
  }

  public void spinShooterWheel(double shooterRPS) {
    shooterLeftMotor.setControl(shooterMotorVelocity.withVelocity(shooterRPS));
    shooterRightMotor.setControl(shooterMotorVelocity.withVelocity(shooterRPS));
  }

  public Command sysIdShooterMotorQuasiCommand(Direction direction) {
    return ShooterMotorSysIdRoutine.quasistatic(direction).withName("SysId Drive Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterMotorDynamCommand(Direction direction) {
    return ShooterMotorSysIdRoutine.dynamic(direction).withName("SysId Drive Quasistatic " + direction)
        .finallyDo(this::stop);
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

  /**
   * Gets the wrist velocity in radians per second
   * 
   * @return wrist velocity in radians per second
   */
  public double getWristVelocity() {
    return Units.rotationsToDegrees(actuatorEncoder.getVelocity());
  }

  public boolean checkWristPosition(double radiansToRotate) {
    return (Math.abs(getWristPosition() - radiansToRotate) <= WRIST_POSITION_OFFSET);
  }

  public boolean checkShooterSpeed(double shooterSpeedGoal) {
    return (Math.abs(getWristVelocity() - shooterSpeedGoal) <= SHOOTER_VELOCITY_OFFSET);
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
    // Set the target position, but move in execute() so feed forward keeps updating
    var newGoal = new TrapezoidProfile.State(target, 0.0);
    if (!Objects.equals(newGoal, goal)) {
      var currentState = new TrapezoidProfile.State(getWristPosition(), 0);
      new TrapezoidProfile(PROFILE_CONSTRAINTS);
    }
    goal = newGoal;
  }

  public void actuatorStop() {
    actuatorMotor.setVoltage(0);
  }
}