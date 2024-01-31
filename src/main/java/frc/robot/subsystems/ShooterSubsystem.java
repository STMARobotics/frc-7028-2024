package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.ACTUATOR_CANCORDER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_AIM;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_POSITION_SLOT_CONFIG;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeftMotor = new TalonFX(DEVICE_ID_LEFT);
  private final TalonFX shooterRightMotor = new TalonFX(DEVICE_ID_RIGHT);

  private final TalonFX actuatorMotor = new TalonFX(DEVICE_ID_AIM);
  private final CANcoder canCoder = new CANcoder(ACTUATOR_CANCORDER);

  private final VelocityTorqueCurrentFOC shooterVelocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC shooterPositionControl = new PositionTorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC actuatorMotorPosition = new PositionTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);

  // SysId routines  
  private SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterLeftMotor.setControl(voltageControl.withOutput(volts.in(Volts)));
        shooterRightMotor.setControl(voltageControl.withOutput(volts.in(Volts)));
      }, null, this));

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

    // Configure aim encoder
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = 0.4;
    canCoder.getConfigurator().apply(canCoderConfig);

    // Configure aim motor
    TalonFXConfiguration actuatorConfig = new TalonFXConfiguration();
    actuatorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    actuatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    actuatorConfig.Feedback.SensorToMechanismRatio = 1.0;
    actuatorConfig.Feedback.RotorToSensorRatio = 12.8;
    actuatorConfig.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    actuatorConfig.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    actuatorConfig.Voltage.PeakForwardVoltage = 8;
    actuatorConfig.Voltage.PeakReverseVoltage = -8;
    actuatorMotor.getConfigurator().apply(actuatorConfig);
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

  /**
   * Returns a new command to rotate the shooter a given distance
   * @param distance distance to rotate, relative to the current position
   * @return new command
   */
  public Command rotateShooterCommand(Measure<Angle> distance) {
    return run(() -> rotateShooterWheels(distance)).finallyDo(this::stopShooter);
  }

  public Command sysIdShooterDyanmicCommand(Direction direction) {
    return shooterSysIdRoutine.dynamic(direction).withName("Shooter dynam " + direction)
        .finallyDo(this::stopShooter);
  }

  public Command sysIdShooterQuasistaticCommand(Direction direction) {
    return shooterSysIdRoutine.quasistatic(direction).withName("Shooter quasi " + direction)
        .finallyDo(this::stopShooter);
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

  private void stopShooter() {
    shooterRightMotor.stopMotor();
    shooterLeftMotor.stopMotor();
  }

}