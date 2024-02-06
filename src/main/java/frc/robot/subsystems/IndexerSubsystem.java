package frc.robot.subsystems;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static frc.robot.Constants.IndexerConstants.BELT_RUN_SPEED;
import static frc.robot.Constants.IndexerConstants.COLOR_NONE;
import static frc.robot.Constants.IndexerConstants.COLOR_NOTE;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_LEFT_MOTOR;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_RIGHT_MOTOR;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.I2C.Port;
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
  private static SparkPIDController leftPidController;
  private static SparkPIDController rightPidController;

  private final ColorSensorV3 indexerColorSensor = new ColorSensorV3(Port.kOnboard);
  private final ColorMatch indexerColorMatch = new ColorMatch();


  public IndexerSubsystem() {
    leftPidController = indexerMotorConfig(leftIndexerMotor, true);
    rightPidController = indexerMotorConfig(rightIndexerMotor, false);

    indexerColorMatch.addColorMatch(COLOR_NOTE);
    indexerColorMatch.addColorMatch(COLOR_NONE);
  }
  
  public boolean shouldContinue() {
    return indexerColorMatch.matchClosestColor(indexerColorSensor.getColor()).color == COLOR_NOTE;
  }

  // Check actual motor values for dash board debugging
  public boolean isActive() {
    return leftIndexerMotor.get() > 0 || rightIndexerMotor.get() > 0;
  }

  public void runIndexer() {
    leftPidController.setReference(BELT_RUN_SPEED, ControlType.kVelocity);
    rightPidController.setReference(BELT_RUN_SPEED, ControlType.kVelocity);
  }

  public void stopIndexer() {
    leftPidController.setReference(0, ControlType.kVelocity);
    rightPidController.setReference(0, ControlType.kVelocity);
  } 

  // Intake is intended to be called multiple times as it's called in "execute" for "IntakeCommand.java"
  public void intake() {
    if (shouldContinue()) {
      runIndexer();
    } else {
      stopIndexer();
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
    indexerLayout.addBoolean("Ring detected", this::shouldContinue);
    indexerLayout.addBoolean("Active motors", this::isActive);
  }

  // Driver tab 
  public void addDriverDashboardWidgets(ShuffleboardTab driverDashboardTab) {
    driverDashboardTab.addBoolean("Ring detected", this::shouldContinue);
    driverDashboardTab.addBoolean("Active indexer motors", this::isActive);
  }

  // Semioperable implementation of smart dashboard, edited by me before I did a small revamp above
  // public void addDashboardWidgets_(ShuffleboardLayout dashboard) {
  //   var detailDashboard = dashboard.getLayout("Detail", BuiltInLayouts.kGrid)
  //       .withProperties(Map.of("Number of columns", 2, "Number of rows", 3)).withPosition(0, 0);
  //   detailDashboard.addBoolean("Detects ring", () -> shouldContinue()).withPosition(0, 0);
  //   detailDashboard.addBoolean("Is active", () -> isActive());
  // }

  // public void addDriverDashboardWidgets_(ShuffleboardTab dashboard) {
  //   dashboard.addBoolean("Ring Position", () -> shouldContinue()).withWidget(BuiltInWidgets.kDial)
  //       .withProperties(Map.of("Min", 0, "Max", 2)).withSize(1, 1).withPosition(8, 3);
  //   var colorSensorLayout = dashboard.getLayout("Indexer", BuiltInLayouts.kGrid)
  //       .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
  //       .withSize(1, 3).withPosition(8, 0);
  //   colorSensorLayout.addBoolean("Shooter", () -> true).withPosition(0, 1);
  //   colorSensorLayout.addBoolean("Intake", () -> true).withPosition(0, 2);

  // }

  private static SparkPIDController indexerMotorConfig(CANSparkMax sparkMax, boolean invert) {
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

  public void fireDonut(Measure<Voltage> volts) {
    leftIndexerMotor.setVoltage(volts.in(Volts));
    rightIndexerMotor.setVoltage(volts.in(Volts));
  }
}
