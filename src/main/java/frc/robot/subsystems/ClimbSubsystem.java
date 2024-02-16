package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Percent;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_LEFT;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_RIGHT;
import static frc.robot.Constants.ClimbConstants.RAMP_RATE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final TalonFX leftWinchMotor = new TalonFX(DEVICE_ID_LEFT, CANIVORE_BUS_NAME);
  private final TalonFX rightWinchMotor = new TalonFX(DEVICE_ID_RIGHT, CANIVORE_BUS_NAME);

  private final DutyCycleOut leftControl = new DutyCycleOut(0.0).withEnableFOC(true);
  private final DutyCycleOut rightControl = new DutyCycleOut(0.0).withEnableFOC(true);

  public ClimbSubsystem() {
    var talonConfig = new TalonFXConfiguration();
    talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = RAMP_RATE;
    talonConfig.MotorOutput.NeutralMode = Brake;

    leftWinchMotor.getConfigurator().apply(talonConfig);
    rightWinchMotor.getConfigurator().apply(talonConfig);
  }

  /**
   * Runs the left climb winch
   * @param output percent output, positive rolls the line IN
   */
  public void runLeftWinch(Measure<Dimensionless> output) {
    leftWinchMotor.setControl(leftControl.withOutput(output.in(Percent)));
  }

  /**
   * Runs the right climb winch
   * @param output percent output, positive rolls the line IN
   */
  public void runRightWinch(Measure<Dimensionless> output) {
    rightWinchMotor.setControl(rightControl.withOutput(output.in(Percent)));
  }

  /**
   * Runs both climb winches
   * @param output percent output, positive rolls the line IN
   */
  public void runWinches(Measure<Dimensionless> output) {
    runLeftWinch(output);
    runRightWinch(output);
  }

  /**
   * Stops both climb winches
   */
  public void stop() {
    leftWinchMotor.stopMotor();
    rightWinchMotor.stopMotor();
  }

}
