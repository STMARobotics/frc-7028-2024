package frc.robot.subsystems;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_LEFT_MOTOR;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_RIGHT_MOTOR;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_INTAKE;
import static frc.robot.Constants.IndexerConstants.THRESHOLD_SHOOTER;

import java.util.Map;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.ColorSensorReader;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class IndexerSubsystem extends SubsystemBase {
  public enum IndexerState {
    noRing,
    intakingRing,
    HasRing,
    processingRing
  }

  public IndexerState indexerState;

  public boolean capturedRing() {
    return (this.indexerState != IndexerState.noRing);
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  private boolean isIntakeSensorTripped() {
    return colorSensorReader.getIntakeValues().proximity > THRESHOLD_INTAKE;
  }

  public boolean isShooterSensorTripped() {
    return colorSensorReader.getShooterValues().proximity > THRESHOLD_SHOOTER;
  }

  public void stop() {
    pidController.setReference(0, ControlType.kVelocity);
  }

  public void load() {
    pidController.setReference(IndexerConstants.BELT_RUN_SPEED, ControlType.kVelocity);
  }
  private final CANSparkMax leftIndexerMotor = new CANSparkMax(DEVICE_ID_INDEXER_LEFT_MOTOR, kBrushless);
  private final CANSparkMax rightIndexerMotor = new CANSparkMax(DEVICE_ID_INDEXER_RIGHT_MOTOR, kBrushless);
  private static SparkPIDController pidController;
  private final ColorSensorReader colorSensorReader = new ColorSensorReader();
  private final Notifier colorSensorNotifier = new Notifier(colorSensorReader);

  private SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
    new SysIdRoutine.Mechanism((volts) -> {
      leftIndexerMotor.setVoltage(volts.in(Volts));
      rightIndexerMotor.setVoltage(volts.in(Volts));
    }, null, this));

  public IndexerSubsystem() {
    indexerMotorConfig(leftIndexerMotor, true);
    indexerMotorConfig(rightIndexerMotor, false);

    colorSensorReader.run();
    // Update the color sensors in the background to prevent loop overrun
    colorSensorNotifier.setName("Color Sensors");
    colorSensorNotifier.startPeriodic(0.02);

  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 3)).withPosition(0, 0);
    detailDashboard.addNumber("Intake Prox", () -> colorSensorReader.getIntakeValues().proximity).withPosition(0, 0);
    detailDashboard.addNumber("Shooter Prox", () -> colorSensorReader.getIntakeValues().proximity).withPosition(0, 0);
    detailDashboard.addString("Ring Type", () -> this.getIndexerState().name()).withPosition(1, 1);

  }

  public void addDriverDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.addBoolean("Ring Position", this::capturedRing).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0, "Max", 2)).withSize(1, 1).withPosition(8, 3);
    var colorSensorLayout = dashboard.getLayout("Indexer", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
        .withSize(1, 3).withPosition(8, 0);
    colorSensorLayout.addBoolean("Shooter", () -> true).withPosition(0, 1);
    colorSensorLayout.addBoolean("Intake", () -> true).withPosition(0, 2);

  }

  public void intake() {
    // sensors return false when something is detected
    if ((!isIntakeSensorTripped() && !isShooterSensorTripped()) || // if all sensors are clear stop the belt
        (isShooterSensorTripped())) { // if fullsensor is tripped OR intake and 2nd sensor are clear
      stop();
    } else {
      load();
    }
  }

  private static SparkPIDController indexerMotorConfig(CANSparkMax sparkMax, boolean invert) {
    sparkMax.restoreFactoryDefaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setOpenLoopRampRate(0.1);
    sparkMax.setClosedLoopRampRate(0.1);
    sparkMax.setInverted(true);
    pidController = sparkMax.getPIDController();
    pidController.setP(IndexerConstants.BELT_kP);
    pidController.setFF(0.00009);
    sparkMax.getEncoder();
    sparkMax.burnFlash();
    sparkMax.setIdleMode(IdleMode.kCoast);
    return pidController;
  }

  public Command sysIdIndexerMotorQuasiCommand(Direction direction) {
    return indexerSysIdRoutine.quasistatic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdIndexerMotorDynamCommand(Direction direction) {
    return indexerSysIdRoutine.dynamic(direction).withName("SysId Deploy Motor Quasistatic " + direction)
        .finallyDo(this::stop);
  }
  public void fireDonut(Measure<Voltage> volts) {
    leftIndexerMotor.setVoltage(volts.in(Volts));
    rightIndexerMotor.setVoltage(volts.in(Volts));

  }

}
