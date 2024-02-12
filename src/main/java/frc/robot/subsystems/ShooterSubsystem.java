package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.ALTITUDE_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_ALTITUDE_CONTROL;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_FOLLOWER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_LEADER;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_SHOOTER_MOTOR_INTAKE;
import static frc.robot.Constants.ShooterConstants.SHOOTER_INTAKE_SLOT_CONFIGS;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SLOT_CONFIGS;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX shooterLeftMotor = new TalonFX(DEVICE_ID_SHOOTER_LEADER, CANIVORE_BUS_NAME);
  private final TalonFX shooterRightMotor = new TalonFX(DEVICE_ID_SHOOTER_FOLLOWER, CANIVORE_BUS_NAME);
  private final TalonFX shooterAltitudeControl = new TalonFX(DEVICE_ID_SHOOTER_ALTITUDE_CONTROL, CANIVORE_BUS_NAME);
  private final TalonFX shooterIntakeMotor = new TalonFX(DEVICE_ID_SHOOTER_MOTOR_INTAKE, CANIVORE_BUS_NAME);

  public boolean hasRing = false;
  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private SysIdRoutine shooterMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterRightMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
        shooterLeftMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private SysIdRoutine altitudeMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterAltitudeControl.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private SysIdRoutine shooterIntakeMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterIntakeMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private final VelocityTorqueCurrentFOC shooterMotorVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
      false);

  private final VelocityTorqueCurrentFOC altitudeControlVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false, false);

  private final VelocityTorqueCurrentFOC IntakeControlVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false,
      false, false);

  public ShooterSubsystem() {
    var altitudeMotorConfig = new TalonFXConfiguration();
    var shooterMotorConfig = new TalonFXConfiguration();
    var intakeMotorConfig = new TalonFXConfiguration();

    intakeMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_INTAKE_SLOT_CONFIGS);
    intakeMotorConfig.MotorOutput.NeutralMode = Coast;
    intakeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    altitudeMotorConfig.Slot0 = Slot0Configs.from(ALTITUDE_SLOT_CONFIGS);
    altitudeMotorConfig.MotorOutput.NeutralMode = Coast;
    altitudeMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    altitudeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_SLOT_CONFIGS);
    shooterMotorConfig.MotorOutput.NeutralMode = Coast;
    shooterMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;

    shooterAltitudeControl.getConfigurator().apply(altitudeMotorConfig);
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterLeftMotor.getConfigurator().apply(shooterMotorConfig);
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterRightMotor.getConfigurator().apply(shooterMotorConfig);
  }

  public void spinShooterWheel() {
    shooterLeftMotor.setControl(shooterMotorVelocity.withVelocity(10));
    shooterRightMotor.setControl(shooterMotorVelocity.withVelocity(10));
  }

  public void altitudeUp() {
    shooterAltitudeControl.setControl(altitudeControlVelocity.withVelocity(1));
  }

  public void altitudeDown() {
    shooterAltitudeControl.setControl(altitudeControlVelocity.withVelocity(-1));;
  }

  public void shootDutyCycle(double speed) {
    shooterRightMotor.set(speed);
    shooterLeftMotor.set(speed);
  }

  public void runIntake() {
    shooterIntakeMotor.setControl(IntakeControlVelocity.withVelocity(3));
  }

  public void stop() {
    shooterRightMotor.stopMotor();
    shooterLeftMotor.stopMotor();
    shooterAltitudeControl.stopMotor();
    shooterIntakeMotor.stopMotor();
  }

  public Command sysIdShooterMotorQuasiCommand(Direction direction) {
    return shooterMotorSysIdRoutine.quasistatic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterMotorDynamCommand(Direction direction) {
    return shooterMotorSysIdRoutine.dynamic(direction).withName("SysId Shooter Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdAltitudeMotorQuasiCommand(Direction direction) {
    return altitudeMotorSysIdRoutine.quasistatic(direction).withName("SysId Altitude Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdAltitudeMotorDynamCommand(Direction direction) {
    return altitudeMotorSysIdRoutine.dynamic(direction).withName("SysId Altitude Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterIntakeMotorQuasiCommand(Direction direction) {
    return shooterIntakeMotorSysIdRoutine.quasistatic(direction)
        .withName("SysId Shooter Intake Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterIntakeMotorDynamCommand(Direction direction) {
    return shooterIntakeMotorSysIdRoutine.dynamic(direction)
        .withName("SysId Shooter Intake Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

}