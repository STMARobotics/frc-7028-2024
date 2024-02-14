package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.IndexerConstants.LEFT_kA;
import static frc.robot.Constants.IndexerConstants.LEFT_kS;
import static frc.robot.Constants.IndexerConstants.LEFT_kV;
import static frc.robot.Constants.IndexerConstants.PORT_ID_FULL_SENSOR;
import static frc.robot.Constants.IndexerConstants.RIGHT_kA;
import static frc.robot.Constants.IndexerConstants.RIGHT_kS;
import static frc.robot.Constants.IndexerConstants.RIGHT_kV;
import static frc.robot.Constants.IndexerConstants.UNLOAD_SPEED;
import static frc.robot.Constants.IndexerConstants.kD;
import static frc.robot.Constants.IndexerConstants.kI;
import static frc.robot.Constants.IndexerConstants.kP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax leftMotor = new CANSparkMax(DEVICE_ID_LEFT, kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(DEVICE_ID_RIGHT, kBrushless);
  private final SparkPIDController leftPidController;
  private final SparkPIDController rightPidController;
  
  private final DigitalInput fullSensor = new DigitalInput(PORT_ID_FULL_SENSOR);
  
  private final SimpleMotorFeedforward leftMotorFeedforward =  new SimpleMotorFeedforward(LEFT_kS, LEFT_kV, LEFT_kA);
  private final SimpleMotorFeedforward rightMotorFeedForward = new SimpleMotorFeedforward(RIGHT_kS, RIGHT_kV, RIGHT_kA);

  private final BooleanConsumer telemetryFunction;

  // SysId routines  
  private final SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((volts) -> {
        leftMotor.setVoltage(volts.in(Volts));
        rightMotor.setVoltage(volts.in(Volts));
      }, null, this));

  public IndexerSubsystem(BooleanConsumer telemetryFunction) {
    this.telemetryFunction = telemetryFunction;
    leftPidController = configureMotor(leftMotor, false);
    rightPidController = configureMotor(rightMotor, true);
  }

  private static SparkPIDController configureMotor(CANSparkMax sparkMax, boolean invert) {
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

  @Override
  public void periodic() {
    if (telemetryFunction != null) {
      telemetryFunction.accept(isFullSensorTripped());
    }
  }

  /**
   * Returns a new command to use the sensors and run the indexer to load a game piece from the intake
   * @return new command
   */
  public Command intakeCommand() {
    return run(this::intake).finallyDo(this::stopNow);
  }

  /**
   * Returns a new command to run the intake backwards to eject a game piece
   * @return new command
   */
  public Command unloadCommand() {
    return run(this::unload).finallyDo(this::stop);
  }

  /**
   * Returns a new command to pass a note from the indexer to the shooter
   * @return new command
   */
  public Command shootCommand() {
    return run(this::shoot).finallyDo(this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /**
   * Returns a new command to run the indexer
   * @param velocity target velocity
   * @return new command
   */
  public Command runCommand(Measure<Velocity<Angle>> velocity) {
    return run(() -> runIndexer(velocity)).finallyDo(this::stop);
  }

  public Command sysIdIndexerDynamicCommand(Direction direction) {
    return indexerSysIdRoutine.dynamic(direction).withName("SysId indexer dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdIndexerQuasistaticCommand(Direction direction) {
    return indexerSysIdRoutine.quasistatic(direction).withName("SysId indexer quasi " + direction)
        .finallyDo(this::stop);
  }

  private void intake() {
    if (isFullSensorTripped()) {
      stopNow();
    } else {
      load();
    }
  }

  /**
   * Returns true when the full sensor is tripped. The falling edge is debounced.
   * @return true if the full sensor is tripped, otherwise false
   */
  public boolean isFullSensorTripped() {
    return fullSensor.get();
  }

  /**
   * Gets the velocity of the left wheels
   * @return velocity of the left wheels
   */
  public Measure<Velocity<Angle>> getLeftVelocity() {
    return Rotations.per(Minute).of(leftMotor.getEncoder().getVelocity());
  }

  /**
   * Gets the velocity of the right wheels
   * @return velocity of right wheels
   */
  public Measure<Velocity<Angle>> getRightVelocity() {
    return Rotations.per(Minute).of(rightMotor.getEncoder().getVelocity());
  }

  public void shoot() {
    runIndexer(IndexerConstants.SHOOT_SPEED);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private void load() {
    runIndexer(IndexerConstants.RUN_SPEED);
  }

  private void unload() {
    leftMotor.set(UNLOAD_SPEED.in(Rotations.per(Minute)));
    rightMotor.set(UNLOAD_SPEED.in(Rotations.per(Minute)));
  }

  private void stopNow() {
    leftPidController.setReference(0, kVelocity);
    rightPidController.setReference(0, kVelocity);
  }

  private void runIndexer(Measure<Velocity<Angle>> velocity) {
    var rpm = velocity.in(Rotations.per(Minute));
    leftPidController.setReference(rpm,  kVelocity, 0, leftMotorFeedforward.calculate(rpm));
    rightPidController.setReference(rpm,  kVelocity, 0, rightMotorFeedForward.calculate(rpm));
  }

}