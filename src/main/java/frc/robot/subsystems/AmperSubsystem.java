package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.AmperConstants.DEVICE_ID_ROLLERS;
import static frc.robot.Constants.AmperConstants.ROLLER_EJECT_VELOCITY;
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

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class AmperSubsystem extends SubsystemBase {
  
  private final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERS, CANIVORE_BUS_NAME);

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
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.Feedback.SensorToMechanismRatio = ROLLER_SENSOR_TO_MECHANISM_RATIO;
    rollerConfig.Slot0 = Slot0Configs.from(ROLLER_SLOT_CONFIGS);

    rollerMotor.getConfigurator().apply(rollerConfig);
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
   * Load a note into the amper
   */
  public void load() {
    runRollers(ROLLER_LOAD_VELOCITY);
  }

  /**
   * Eject a note out the bottom of the amper
   */
  public void eject() {
    runRollers(ROLLER_EJECT_VELOCITY);
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
