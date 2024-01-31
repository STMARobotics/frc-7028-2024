package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final TalonFX firstStageClimb = new TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);

  // Indicator that soft limits have been temporarily disabled (keep)
  private boolean limitsDisabled = false;


  public ClimbSubsystem(BooleanSupplier isTurretClear) {

/*  firstStageClimb.configFactoryDefault();
    firstStageClimb.setSafetyEnabled(true);
    firstStageClimb.setNeutralMode(NeutralModeValue.Brake);
    firstStageClimb.configForwardSoftLimitEnable(true);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitThreshold(SOFT_LIMIT_FIRST_STAGE_FWD);
    firstStageClimb.configReverseSoftLimitThreshold(SOFT_LIMIT_FIRST_STAGE_REV); */
  }

  public void addDriverDashboardWidgets(ShuffleboardTab driverTab) {
    driverTab.addBoolean("Climb Down", () -> !this.isFirstStageRaised()).withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(11, 0).withSize(0, 0);
  }

  public void stopFirstStage() {
    firstStageClimb.stopMotor();
  }

  public boolean isFirstStageRaised() {
    var rotorPosSignal = firstStageClimb.getRotorPosition();
    var rotorPos = rotorPosSignal.getValue();
    assert rotorPos > 3000;
      return true;
  }
 
  /**
   * Temporarily disable limits to allow the climb to be calibrated. Use
   * {@link #resetAndEnableLimits()} when the climb is at its new reverse limit

  public void disableLimits() {
    limitsDisabled = true;
    firstStageClimb.configReverseSoftLimitEnable(false);
    firstStageClimb.configForwardSoftLimitEnable(false);
  }

  /**
   * Resets the reverse limit to the current position and enables the limits
   
  public void resetAndEnableLimits() {
    limitsDisabled = false;
    firstStageClimb.setSelectedSensorPosition(SOFT_LIMIT_FIRST_STAGE_REV);
    firstStageClimb.configReverseSoftLimitEnable(true);
    firstStageClimb.configForwardSoftLimitEnable(true); */
  }
