package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.ACTUATOR_CANCORDER;
import static frc.robot.Constants.ShooterConstants.ACTUATOR_FALCON;
import static frc.robot.Constants.ShooterConstants.SHOOTER_DONUT_POSITION_CONTROl;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_CONTROl;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final double ENCODER_CPR = 2048;
  private final TalonFX shooterLeftMotor = new TalonFX(SHOOTER_DONUT_POSITION_CONTROl);
  private final TalonFX shooterRightMotor = new TalonFX(SHOOTER_VELOCITY_CONTROl);
  private final TalonFX actuatorMotor = new TalonFX(ACTUATOR_FALCON);
  private final CANcoder canCoder = new CANcoder(ACTUATOR_CANCORDER);
  // creates new talonfx devices and sets them to the device Ids set in
  // Constants.java

  private boolean isActiveStopped = false;

  private final VelocityTorqueCurrentFOC shooterMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);
  private final PositionTorqueCurrentFOC actuatorMotorVelocity = new PositionTorqueCurrentFOC(0, 0, 0, 1, false,
      false, false);

  public ShooterSubsystem() {
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
    shooterMotorConfig.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    shooterMotorConfig.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = 0.4;

    TalonFXConfiguration actuatorConfig = new TalonFXConfiguration();
    actuatorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    actuatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    actuatorConfig.Feedback.SensorToMechanismRatio = 1.0;
    actuatorConfig.Feedback.RotorToSensorRatio = 12.8;
    actuatorConfig.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    actuatorConfig.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    actuatorConfig.Voltage.PeakForwardVoltage = 8;
    actuatorConfig.Voltage.PeakReverseVoltage = -8;

    shooterLeftMotor.setInverted(true);

    shooterRightMotor.setNeutralMode(NeutralModeValue.Brake);
    shooterLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    canCoder.getConfigurator().apply(canCoderConfig);
    shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
    shooterRightMotor.getConfigurator().apply(shooterMotorConfig);
    actuatorMotor.getConfigurator().apply(actuatorConfig);
  }

  public void spinShooterWheel(int x) {
    shooterLeftMotor.setControl(shooterMotorVelocity.withVelocity(x));
    shooterRightMotor.setControl(shooterMotorVelocity.withVelocity(x));
    actuatorMotor.setControl(actuatorMotorVelocity.withVelocity(x));
  }

  public void activeStop() {
    if (!isActiveStopped) {
      shooterLeftMotor.setControl(actuatorMotorVelocity.withVelocity(0));
      shooterRightMotor.setControl(actuatorMotorVelocity.withVelocity(0));
    }
    isActiveStopped = true;
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

  public static double edgesPerDecisecToRPS(double edgesPerDecisec) {
    var rotationsPerDecisecond = edgesPerDecisec / ENCODER_CPR;
    return rotationsPerDecisecond * 10;
  }
}