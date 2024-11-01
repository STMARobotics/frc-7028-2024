package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.AMP_BOTTOM_VELOCITY;
import static frc.robot.Constants.ShooterConstants.AMP_TOP_VELOCITY;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_BOTTOM;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_TOP;
import static frc.robot.Constants.ShooterConstants.ERROR_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.PEAK_FORWARD_CURRENT;
import static frc.robot.Constants.ShooterConstants.PEAK_REVERSE_CURRENT;
import static frc.robot.Constants.ShooterConstants.REVERSE_VELOCITY;
import static frc.robot.Constants.ShooterConstants.SENSOR_TO_MECHANISM_RATIO;
import static frc.robot.Constants.ShooterConstants.SLOT_CONFIG_BOTTOM;
import static frc.robot.Constants.ShooterConstants.SLOT_CONFIG_TOP;
import static frc.robot.Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

/** Subsystem for the shooter mechanism */
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX topMotor = new TalonFX(DEVICE_ID_TOP, CANIVORE_BUS_NAME);
  private final TalonFX bottomMotor = new TalonFX(DEVICE_ID_BOTTOM, CANIVORE_BUS_NAME);

  private final VelocityTorqueCurrentFOC topControl = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC bottomControl = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> bottomVelocity;
  private final StatusSignal<AngularAcceleration> bottomAcceleration;
  private final StatusSignal<AngularVelocity> topVelocity;
  private final StatusSignal<AngularAcceleration> topAcceleration;

  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);

  // SysId routine - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second), Volts.of(25), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism(
          (amps) -> {
            topMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
            bottomMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
          },
          null,
          this));

  public ShooterSubsystem() {
    // Configure shooter motors
    var motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Slot0Configs.from(SLOT_CONFIG_BOTTOM);
    motorConfig.MotorOutput.NeutralMode = Coast;
    motorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_FORWARD_CURRENT.in(Amps);
    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = PEAK_REVERSE_CURRENT.in(Amps);

    bottomMotor.getConfigurator().apply(motorConfig);
    motorConfig.Slot0 = Slot0Configs.from(SLOT_CONFIG_TOP);
    topMotor.getConfigurator().apply(motorConfig);

    topVelocity = topMotor.getVelocity();
    topAcceleration = topMotor.getAcceleration();
    bottomVelocity = bottomMotor.getVelocity();
    bottomAcceleration = bottomMotor.getAcceleration();
  }

  public Command sysIdShooterDynamicCommand(Direction direction) {
    return sysIdRoutine
        .dynamic(direction)
        .withName("Shooter dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterQuasistaticCommand(Direction direction) {
    return sysIdRoutine
        .quasistatic(direction)
        .withName("Shooter quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the shooter wheels at a given velocity
   *
   * @param velocity velocity for both sets of wheels
   */
  public void spinShooterWheels(AngularVelocity velocity) {
    topMotor.setControl(topControl.withVelocity(velocity));
    bottomMotor.setControl(bottomControl.withVelocity(velocity));
  }

  /**
   * Spins the shooter wheels at a given velocity
   *
   * @param topVelocity velocity for top set of wheels
   * @param bottomVelocity velocity for bottom set of wheels
   */
  public void spinShooterWheels(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    topMotor.setControl(topControl.withVelocity(topVelocity));
    bottomMotor.setControl(bottomControl.withVelocity(bottomVelocity));
  }

  /**
   * Spin up the wheels to a given velocity
   *
   * @param shooterVelocity
   */
  public void prepareToShoot(AngularVelocity shooterVelocity) {
    spinShooterWheels(shooterVelocity);
  }

  /** Spins the wheels to amper velocity */
  public void prepareToAmp() {
    spinShooterWheels(AMP_TOP_VELOCITY, AMP_BOTTOM_VELOCITY);
  }

  /** Spins the wheels in reverse */
  public void reverse() {
    spinShooterWheels(REVERSE_VELOCITY);
  }

  /**
   * Checks if the shooter is at the taget velocity
   *
   * @return true if the shooter is at the target velocity
   */
  public boolean isReadyToShoot() {
    var errorToleranceRPS = ERROR_TOLERANCE.in(RotationsPerSecond);
    BaseStatusSignal.refreshAll(bottomVelocity, bottomAcceleration, topVelocity, topAcceleration);

    var compensatedTop = BaseStatusSignal.getLatencyCompensatedValue(topVelocity, topAcceleration);
    var compensatedBottom = BaseStatusSignal.getLatencyCompensatedValue(bottomVelocity, bottomAcceleration);

    return Math.abs(compensatedBottom.in(RotationsPerSecond) - bottomControl.Velocity) < errorToleranceRPS
        && Math.abs(compensatedTop.in(RotationsPerSecond) - topControl.Velocity) < errorToleranceRPS;
  }

  /** Stops the shooter */
  public void stop() {
    bottomMotor.stopMotor();
    topMotor.stopMotor();
  }
}
