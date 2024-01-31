package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.COLOR_NONE;
import static frc.robot.Constants.IndexerConstants.COLOR_NOTE;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_INTAKE;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_SPACE;
import static frc.robot.Constants.IndexerConstants.kFF;
import static frc.robot.Constants.IndexerConstants.kP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.ColorSensorReader;

public class IndexerSubsystem extends SubsystemBase {

  private final CANSparkMax indexerLeftMotor = new CANSparkMax(DEVICE_ID_LEFT, kBrushless);
  private final CANSparkMax indexerRightMotor = new CANSparkMax(DEVICE_ID_RIGHT, kBrushless);
  private final SparkPIDController leftPidController;
  private final SparkPIDController rightPidController;
  
  private final ColorSensorReader colorSensorReader = new ColorSensorReader();
  private final Notifier colorSensorNotifier = new Notifier(colorSensorReader);
  private final Debouncer fullSensorDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  private ColorMatch colorMatch = new ColorMatch();

  // SysId routines  
  private SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((volts) -> {
        indexerLeftMotor.setVoltage(volts.in(Volts));
        indexerRightMotor.setVoltage(volts.in(Volts));
      }, null, this));

  public IndexerSubsystem() {
    leftPidController = configureMotor(indexerLeftMotor, false);
    rightPidController = configureMotor(indexerRightMotor, true);
    
    colorSensorReader.run();
    // Update the color sensors in the background to prevent loop overrun
    colorSensorNotifier.setName("Color Sensors");
    colorSensorNotifier.startPeriodic(0.02);

    colorMatch.addColorMatch(COLOR_NOTE);
    colorMatch.addColorMatch(COLOR_NONE);
  }

  private SparkPIDController configureMotor(CANSparkMax sparkMax, boolean invert) {
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

  public Command sysIdIndexerDynamicCommand(Direction direction) {
    return shooterSysIdRoutine.dynamic(direction).withName("SysId indexer dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdIndexerQuasistaticCommand(Direction direction) {
    return shooterSysIdRoutine.quasistatic(direction).withName("SysId indexer quasi " + direction)
        .finallyDo(this::stop);
  }

  private void intake() {
    // TODO adjust this logic to load the note all the way to the top sensor, since we can only hold one
    // Also, we probably don't want to automate running based on an intake sensor, we'll probably want to run the whole
    // time the intake wheels are running
    // sensors return false when something is detected
    if ((!isIntakeSensorTripped() && !isSpacerSensorTripped() && !isFullSensorTripped()) || //if all sensors are clear stop the belt
        (isFullSensorTripped() || (!isSpacerSensorTripped() && !isIntakeSensorTripped()))) {  //if fullsensor is tripped OR intake and 2nd sensor are clear
      stop();
    } else {
      load();
    }
  }

  /**
   * Returns the color detected by the intake sensor
   * @return the color detected
   */
  public Color getIntakeColor() {
    return colorMatch.matchClosestColor(colorSensorReader.getIntakeValues().color).color;
  }

  /**
   * Returns the color detected by the spacer sensor
   * @return the color detected
   */
  public Color getSpacerColor() {
     return colorMatch.matchClosestColor(colorSensorReader.getSpacerValues().color).color;
  }

  /**
   * Returns the color detected by the full sensor
   * @return the color detected
   */
  public Color getFullColor() {
    return colorMatch.matchClosestColor(colorSensorReader.getFullValues().color).color;
  }

  /**
   * Returns true when the full sensor is tripped. The falling edge is debounced.
   * @return true if the full sensor is tripped, otherwise false
   */
  public boolean isFullSensorTripped() {
    var sensorTripped = colorMatch.matchClosestColor(colorSensorReader.getFullValues().color).color != COLOR_NONE;
    return fullSensorDebouncer.calculate(sensorTripped);
  }

  private boolean isIntakeSensorTripped() {
    return colorSensorReader.getIntakeValues().proximity > THRESHOLD_INTAKE;
  }

  public boolean isSpacerSensorTripped() {
    return colorSensorReader.getSpacerValues().proximity > THRESHOLD_SPACE;
  }

  private void load() {
    leftPidController.setReference(IndexerConstants.RUN_SPEED, kVelocity);
    rightPidController.setReference(IndexerConstants.RUN_SPEED, kVelocity);
  }

  private void unload() {
    indexerLeftMotor.set(-.5);
    indexerRightMotor.set(-.5);
  }

  private void shoot() {
    leftPidController.setReference(IndexerConstants.SHOOT_SPEED, kVelocity);
    rightPidController.setReference(IndexerConstants.SHOOT_SPEED, kVelocity);
  }

  private void stop() {
    indexerLeftMotor.stopMotor();
    indexerRightMotor.stopMotor();
  }

}