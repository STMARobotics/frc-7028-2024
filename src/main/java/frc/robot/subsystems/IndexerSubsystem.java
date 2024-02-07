package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.COLOR_NONE;
import static frc.robot.Constants.IndexerConstants.COLOR_NOTE;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.IndexerConstants.kFF;
import static frc.robot.Constants.IndexerConstants.kP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexerLeftMotor = new CANSparkMax(DEVICE_ID_LEFT, kBrushless);
  private final CANSparkMax indexerRightMotor = new CANSparkMax(DEVICE_ID_RIGHT, kBrushless);
  private final SparkPIDController leftPidController;
  private final SparkPIDController rightPidController;
  
  private final ColorSensorV3 colorSensorV3 = new ColorSensorV3(Port.kOnboard);
  private final Debouncer fullSensorDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  private final ColorMatch colorMatch = new ColorMatch();

  // SysId routines  
  private final SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((volts) -> {
        indexerLeftMotor.setVoltage(volts.in(Volts));
        indexerRightMotor.setVoltage(volts.in(Volts));
      }, null, this));

  public IndexerSubsystem() {
    leftPidController = configureMotor(indexerLeftMotor, false);
    rightPidController = configureMotor(indexerRightMotor, true);

    colorMatch.addColorMatch(COLOR_NOTE);
    colorMatch.addColorMatch(COLOR_NONE);
  }

  private static SparkPIDController configureMotor(CANSparkMax sparkMax, boolean invert) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setIdleMode(kCoast);
    sparkMax.setOpenLoopRampRate(0.1);
    sparkMax.setClosedLoopRampRate(0.1);
    sparkMax.setInverted(invert);
    var pidController = sparkMax.getPIDController();
    pidController.setP(kP);
    pidController.setFF(kFF);
    sparkMax.burnFlash();
    return pidController;
  }

  /**
   * Returns a new command to use the sensors and run the indexer to load a game piece from the intake
   * @return new command
   */
  public Command intakeCommand() {
    return run(this::intake).finallyDo(this::stop);
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
      stop();
    } else {
      load();
    }
  }

  /**
   * Returns the color detected by the full sensor
   * @return the color detected
   */
  public Color getFullColor() {
    return colorMatch.matchClosestColor(colorSensorV3.getColor()).color;
  }

  /**
   * Returns true when the full sensor is tripped. The falling edge is debounced.
   * @return true if the full sensor is tripped, otherwise false
   */
  public boolean isFullSensorTripped() {
    var sensorTripped = colorMatch.matchClosestColor(colorSensorV3.getColor()).color != COLOR_NONE;
    return fullSensorDebouncer.calculate(sensorTripped);
  }

  private void load() {
    runIndexer(IndexerConstants.RUN_SPEED);
  }

  private void unload() {
    indexerLeftMotor.set(-.35);
    indexerRightMotor.set(-.35);
  }

  private void shoot() {
    runIndexer(IndexerConstants.SHOOT_SPEED);
  }

  private void stop() {
    indexerLeftMotor.stopMotor();
    indexerRightMotor.stopMotor();
  }

  private void runIndexer(Measure<Velocity<Angle>> velocity) {
    leftPidController.setReference(velocity.in(RotationsPerSecond), kVelocity);
    rightPidController.setReference(velocity.in(RotationsPerSecond), kVelocity);
  }

}