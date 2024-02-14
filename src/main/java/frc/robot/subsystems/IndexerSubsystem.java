package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_LEFT_MOTOR;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_RIGHT_MOTOR;
import static frc.robot.Constants.IndexerConstants.LEFT_kA;
import static frc.robot.Constants.IndexerConstants.LEFT_kS;
import static frc.robot.Constants.IndexerConstants.LEFT_kV;
import static frc.robot.Constants.IndexerConstants.RIGHT_kA;
import static frc.robot.Constants.IndexerConstants.RIGHT_kS;
import static frc.robot.Constants.IndexerConstants.RIGHT_kV;
import static frc.robot.Constants.IndexerConstants.kD;
import static frc.robot.Constants.IndexerConstants.kI;
import static frc.robot.Constants.IndexerConstants.kP;
import static frc.robot.Constants.IntakeConstants.PORT_ID_DONUT_SENSOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax leftIndexerMotor = new CANSparkMax(DEVICE_ID_INDEXER_LEFT_MOTOR, kBrushless);
  private final CANSparkMax rightIndexerMotor = new CANSparkMax(DEVICE_ID_INDEXER_RIGHT_MOTOR, kBrushless);
  private SparkPIDController leftPidController;
  private SparkPIDController rightPidController;
  private DigitalInput donutSensor = new DigitalInput(PORT_ID_DONUT_SENSOR);

  private final SimpleMotorFeedforward leftMotorFeedforward = new SimpleMotorFeedforward(LEFT_kS, LEFT_kV, LEFT_kA);
  private final SimpleMotorFeedforward rightMotorFeedForward = new SimpleMotorFeedforward(RIGHT_kS, RIGHT_kV, RIGHT_kA);

  public IndexerSubsystem() {
    leftPidController = indexerMotorConfig(leftIndexerMotor, false);
    rightPidController = indexerMotorConfig(rightIndexerMotor, true);

  }

  public boolean hasDonut() {
    return donutSensor.get();
  }

  public boolean isActive() {
    return leftIndexerMotor.get() > 0 || rightIndexerMotor.get() > 0;
  }

  public void runIndexer() {
    leftIndexerMotor.setVoltage(3);
    rightIndexerMotor.setVoltage(3);
    // leftPidController.setReference(BELT_RUN_SPEED, ControlType.kVelocity);
    // rightPidController.setReference(BELT_RUN_SPEED, ControlType.kVelocity);
  }

  public void stopIndexer() {
    leftIndexerMotor.setVoltage(0);
    rightIndexerMotor.setVoltage(0);
  }

  public void intake() {
    if (hasDonut()) {
      stopIndexer();
    } else {
      load();
    }
  }

  private SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        leftIndexerMotor.setVoltage(volts.in(Volts));
        rightIndexerMotor.setVoltage(volts.in(Volts));
      }, null, this));

  public Command sysIdIndexerMotorQuasiCommand(Direction direction) {
    return indexerSysIdRoutine.quasistatic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stopIndexer);
  }

  public Command sysIdIndexerMotorDynamCommand(Direction direction) {
    return indexerSysIdRoutine.dynamic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stopIndexer);
  }

  // Subsystem tab
  public void addSubsystemDashboardWidgets(ShuffleboardLayout indexerLayout) {
    indexerLayout.addBoolean("Ring detected", this::hasDonut);
    indexerLayout.addBoolean("Active motors", this::isActive);
  }

  // Driver tab
  public void addDriverDashboardWidgets(ShuffleboardTab driverDashboardTab) {
    driverDashboardTab.addBoolean("Ring detected", this::hasDonut);
    driverDashboardTab.addBoolean("Active indexer motors", this::isActive);
  }

  private static SparkPIDController indexerMotorConfig(CANSparkMax sparkMax, boolean invert) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setIdleMode(kBrake);
    sparkMax.setOpenLoopRampRate(0.1);
    sparkMax.setClosedLoopRampRate(0.1);
    sparkMax.setInverted(invert);
    var pidController = sparkMax.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    sparkMax.burnFlash();
    return pidController;
  }

  public void fireDonut(Measure<Voltage> volts) {
    leftIndexerMotor.setVoltage(volts.in(Volts));
    rightIndexerMotor.setVoltage(volts.in(Volts));
  }

  public void stop() {
    leftIndexerMotor.setVoltage(0);
    rightIndexerMotor.setVoltage(0);
  }

  public void load() {
    runIndexer(IndexerConstants.RUN_SPEED);
  }

  public void shoot() {
    runIndexer(IndexerConstants.SHOOT_SPEED);
  }

  private void runIndexer(Measure<Velocity<Angle>> velocity) {
    var rpm = velocity.in(Rotations.per(Minute));
    leftPidController.setReference(rpm, kVelocity, 0, leftMotorFeedforward.calculate(rpm));
    rightPidController.setReference(rpm, kVelocity, 0, rightMotorFeedForward.calculate(rpm));
  }
}