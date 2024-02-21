package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_BOTTOM;
import static frc.robot.Constants.ShooterConstants.DEVICE_ID_TOP;
import static frc.robot.Constants.ShooterConstants.SHOOTER_ERROR_TOLERANCE;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SENSOR_TO_MECHANISM_RATIO;
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
  private final TalonFX shooterTopMotor = new TalonFX(DEVICE_ID_TOP, CANIVORE_BUS_NAME);
  private final TalonFX shooterBottomMotor = new TalonFX(DEVICE_ID_BOTTOM, CANIVORE_BUS_NAME);

  private final VelocityTorqueCurrentFOC shooterVelocityControl = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Double> shooterBottomVelocity;
  private final StatusSignal<Double> shooterTopVelocity;

  private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
  
  // SysId routine - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(3.0).per(Seconds.of(1)), Volts.of(25), null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((amps) -> {
        shooterTopMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
        shooterBottomMotor.setControl(sysIdControl.withOutput(amps.in(Volts)));
      }, null, this));

  public ShooterSubsystem() {
    // Configure shooter motors
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_BOTTOM);
    shooterMotorConfig.MotorOutput.NeutralMode = Coast;
    shooterMotorConfig.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECHANISM_RATIO;
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterBottomMotor.getConfigurator().apply(shooterMotorConfig);
    shooterMotorConfig.Slot0 = Slot0Configs.from(SHOOTER_VELOCITY_SLOT_CONFIG_TOP);
    shooterTopMotor.getConfigurator().apply(shooterMotorConfig);

    shooterTopVelocity = shooterTopMotor.getVelocity();
    shooterBottomVelocity = shooterBottomMotor.getVelocity();
  }

  public Command sysIdShooterDynamicCommand(Direction direction) {
    return shooterSysIdRoutine.dynamic(direction).withName("Shooter dynam " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdShooterQuasistaticCommand(Direction direction) {
    return shooterSysIdRoutine.quasistatic(direction).withName("Shooter quasi " + direction)
        .finallyDo(this::stop);
  }

  public void spinShooterWheels(Measure<Velocity<Angle>> velocity) {
    var targetVelocity = velocity.in(RotationsPerSecond);
    shooterTopMotor.setControl(shooterVelocityControl.withVelocity(targetVelocity));
    shooterBottomMotor.setControl(shooterVelocityControl.withVelocity(targetVelocity));
  }

  /**
   * Spin up the wheels to a given velocity
   * @param shooterVelocity
   */
  public void prepareToShoot(Measure<Velocity<Angle>> shooterVelocity) {
    spinShooterWheels(shooterVelocity);
  }

  /**
   * Checks if the shooter is at the taget velocity
   * @return true if the shooter is at the target velocity
   */
  public boolean isReadyToShoot() {
    var errorToleranceRPS = SHOOTER_ERROR_TOLERANCE.in(RotationsPerSecond);
    BaseStatusSignal.refreshAll(shooterBottomVelocity, shooterTopVelocity);

    return Math.abs(shooterBottomVelocity.getValueAsDouble() - shooterVelocityControl.Velocity) < errorToleranceRPS
        && Math.abs(shooterTopVelocity.getValueAsDouble() - shooterVelocityControl.Velocity) < errorToleranceRPS;
  }

  /**
   * Stops the shooter
   */
  public void stop() {
    shooterBottomMotor.stopMotor();
    shooterTopMotor.stopMotor();
  }

}