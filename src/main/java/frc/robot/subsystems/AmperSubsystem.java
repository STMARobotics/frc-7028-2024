package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.AmperConstants.DEVICE_ID_NOTE_SENSOR;
import static frc.robot.Constants.AmperConstants.DEVICE_ID_ROLLERS;
import static frc.robot.Constants.AmperConstants.NOTE_SENSOR_DISTANCE_THRESHOLD;
import static frc.robot.Constants.AmperConstants.ROLLER_INTAKE_VELOCITY;
import static frc.robot.Constants.AmperConstants.ROLLER_LOAD_VELOCITY;
import static frc.robot.Constants.AmperConstants.ROLLER_SCORE_VELOCITY;
import static frc.robot.Constants.AmperConstants.ROLLER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.AmperConstants.ROLLER_SLOT_CONFIGS;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/**
 * Subsystem for the amper mechanism
 */
public class AmperSubsystem extends SubsystemBase {
  
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERS, CANIVORE_BUS_NAME);

  private final LaserCan noteSensor = new LaserCan(DEVICE_ID_NOTE_SENSOR);

  // Motor request objects
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
  
  // SysId routine - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(5).per(Seconds.of(1)), Volts.of(30), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((amps) -> rollerMotor.setControl(sysIdControl.withOutput(amps.in(Volts))), null, this));
    
  public AmperSubsystem() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_SENSOR_TO_MECHANISM_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerConfig);
    
    // Configure the note sensor
    try {
      noteSensor.setRangingMode(RangingMode.SHORT);
      noteSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      DriverStation.reportError("Failed to confgure amper LaserCAN: " + e.getMessage(), false);
    }
  }

  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("SysId amper dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("SysId amper quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Checks in the amper has a note loaded
   * @return true if the amper has a note, otherwise false
   */
  public boolean hasNote() {
    var measure = noteSensor.getMeasurement();
    if (measure == null || measure.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return false;
    }
    return measure.distance_mm < NOTE_SENSOR_DISTANCE_THRESHOLD.in(Millimeters);
  }

  /**
   * Intake a note into the robot, essentially just passing it under the amper.
   */
  public void intake() {
    runRollers(ROLLER_INTAKE_VELOCITY);
  }

  /**
   * Load a note into the amper
   */
  public void load() {
    runRollers(ROLLER_LOAD_VELOCITY);
  }

  /**
   * Score a note out the top of the amper
   */
  public void score() {
    runRollers(ROLLER_SCORE_VELOCITY);
  }

  /**
   * Run the amper rollers at a given velocity
   * @param velocity velocity to run the rollers
   */
  public void runRollers(Measure<Velocity<Angle>> velocity) {
    rollerMotor.setControl(rollerControl.withVelocity(velocity.in(RotationsPerSecond)));
  }

  /**
   * Stop the rollers
   */
  public void stop() {
    rollerMotor.stopMotor();
  }

}
