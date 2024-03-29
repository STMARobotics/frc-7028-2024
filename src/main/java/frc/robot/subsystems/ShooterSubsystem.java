package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_BOTTOM;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_TOP;
import static frc.robot.Constants.ShooterConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.ShooterConstants.SHOOTER_ERROR_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM;
import static frc.robot.Constants.ShooterConstants.SHOOTER_VELOCITY_SLOT_CONFIG_TOP;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/**
 * Subsystem for the shooter mechanism
 */
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX topMotor = new TalonFX(DEVICE_ID_TOP, CANIVORE_BUS_NAME);
  private final TalonFX bottomMotor = new TalonFX(DEVICE_ID_BOTTOM, CANIVORE_BUS_NAME);

  private final VelocityTorqueCurrentFOC topControl = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC bottomControl = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Double> bottomVelocity;
  private final StatusSignal<Double> topVelocity;

  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
  
  // SysId routine - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(3.0).per(Seconds.of(1)), Volts.of(25), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((amps) -> {
        topMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
        bottomMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
      }, null, this));

  public ShooterSubsystem() {
    // Configure shooter motors
    var motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM);
    motorConfig.MotorOutput.NeutralMode = Coast;
    motorConfig.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECHANISM_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = SHOOTER_SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = SHOOTER_STATOR_CURRENT_LIMIT;
    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SHOOTER_STATOR_CURRENT_LIMIT / 2;

    bottomMotor.getConfigurator().apply(motorConfig);
    motorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_TOP);
    topMotor.getConfigurator().apply(motorConfig);

    topVelocity = topMotor.getVelocity();
    bottomVelocity = bottomMotor.getVelocity();
  }

  public Command sysIdShooterDynamicCommand(Direction direction) {
    return sysIdRoutine.dynamic(direction).withName("Shooter dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterQuasistaticCommand(Direction direction) {
    return sysIdRoutine.quasistatic(direction).withName("Shooter quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the shooter wheels at a given velocity
   * @param velocity velocity for both sets of wheels
   */
  public void spinShooterWheels(Measure<Velocity<Angle>> velocity) {
    var targetVelocity = velocity.in(RotationsPerSecond);
    topMotor.setControl(topControl.withVelocity(targetVelocity));
    bottomMotor.setControl(bottomControl.withVelocity(targetVelocity));
  }

  /**
   * Spins the shooter wheels at a given velocity
   * @param topVelocity velocity for top set of wheels
   * @param bottomVelocity velocity for bottom set of wheels
   */
  public void spinShooterWheels(Measure<Velocity<Angle>> topVelocity, Measure<Velocity<Angle>> bottomVelocity) {
    topMotor.setControl(topControl.withVelocity(topVelocity.in(RotationsPerSecond)));
    bottomMotor.setControl(bottomControl.withVelocity(bottomVelocity.in(RotationsPerSecond)));
  }

  /**
   * Spin up the wheels to a given velocity
   * @param shooterVelocity
   */
  public void prepareToShoot(Measure<Velocity<Angle>> shooterVelocity) {
    spinShooterWheels(shooterVelocity);
  }

  public void reverse() {
    spinShooterWheels(REVERSE_VELOCITY);
  }

  /**
   * Checks if the shooter is at the taget velocity
   * @return true if the shooter is at the target velocity
   */
  public boolean isReadyToShoot() {
    var errorToleranceRPS = SHOOTER_ERROR_TOLERANCE.in(RotationsPerSecond);
    BaseStatusSignal.refreshAll(bottomVelocity, topVelocity);

    return Math.abs(bottomVelocity.getValueAsDouble() - bottomControl.Velocity) < errorToleranceRPS
        && Math.abs(topVelocity.getValueAsDouble() - topControl.Velocity) < errorToleranceRPS;
  }

  /**
   * Stops the shooter
   */
  public void stop() {
    bottomMotor.stopMotor();
    topMotor.stopMotor();
  }

}