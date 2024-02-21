
package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimbConstants.CHAIN_HEIGHT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SLOT_CONFIGS;
import static frc.robot.Constants.ElevatorConstants.MOTOR_ENCODER_POSITION_COEFFICIENT;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.sysid.SysIdRoutineSignalLogger;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX climbLeftArm = new TalonFX(ClimbConstants.LEFT_CLIMB_ARM_ID);
  private final TalonFX climbRightArm = new TalonFX(ClimbConstants.RIGHT_CLIMB_ARM_ID);
  private BooleanSupplier isTurretClear;

  private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private SysIdRoutine leftClimbMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterRightMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
        shooterLeftMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  private SysIdRoutine altitudeMotorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, SysIdRoutineSignalLogger.logState()),
      new SysIdRoutine.Mechanism((volts) -> {
        shooterAltitudeControl.setControl(voltageRequest.withOutput(volts.in(Volts)));
      }, null, this));

  public ClimbSubsystem(BooleanSupplier isTurretClear) {
    this.isTurretClear = isTurretClear;

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    climbConfig.Slot0 = Slot0Configs.from(CLIMB_SLOT_CONFIGS);
    climbConfig.MotorOutput.NeutralMode = Brake;
    climbConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    
    climbLeftArm.getConfigurator().apply(climbConfig);
    climbRightArm.getConfigurator().apply(climbConfig);

    climbLeftArm.setInverted(true);
    climbRightArm.setInverted(false);
  }

  public void moveToPosition(double meters) {
    climbLeftArm.setPosition(metersToMotorPosition(meters));
    climbRightArm.setPosition(metersToMotorPosition(meters));
  }

  public void climbUp(){
    if (isTurretClear.getAsBoolean()){
    climbLeftArm.setPosition(metersToMotorPosition(CHAIN_HEIGHT));
    climbRightArm.setPosition(metersToMotorPosition(CHAIN_HEIGHT));
    }
  else {
    climbLeftArm.stopMotor();
    }
  }

  public void climbDown(){
    if (isTurretClear.getAsBoolean()){
    climbLeftArm.setPosition(metersToMotorPosition(0));
    climbRightArm.setPosition(metersToMotorPosition(0));
    }
  else {
    climbLeftArm.stopMotor();
    }
  }

  public void stopClimb() {
    climbLeftArm.stopMotor();
    climbRightArm.stopMotor();
  }

  public double getClimbPosition(){
    var climbPositionSignal = climbLeftArm.getPosition();
    var climbPosition = climbPositionSignal.getValue();
    return motorPositionToMeters(climbPosition);
  }

  public boolean isClimbRaised(){
    var climbPositionSignal = climbLeftArm.getPosition();
    var climbPosition = climbPositionSignal.getValue();
    return motorPositionToMeters(climbPosition) >= CHAIN_HEIGHT;
  }

  private static double motorPositionToMeters(double motorPosition) {
    return (motorPosition * MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  private static double metersToMotorPosition(double positionMeters) {
    return (positionMeters / MOTOR_ENCODER_POSITION_COEFFICIENT);
  }

  public Command sysIdLeftClimbMotorQuasiCommand(Direction direction) {
  return leftClimbMotorSysIdRoutine.quasistatic(direction).withName("SysId Climb Motor Quasistatic " + direction)
      .finallyDo(this::stop);
  }

  public Command sysIdRightClimbMotorQuasiCommand(Direction direction) {
    return rightClimbMotorSysIdRoutine.quasistatic(direction)
        .withName("SysId Shooter Intake Motors Quasistatic " + direction)
        .finallyDo(this::stop);
  }

}
