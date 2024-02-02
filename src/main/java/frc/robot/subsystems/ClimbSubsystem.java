package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.DEVICE_ID_FIRST_STAGE_CLIMB;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  
  private final TalonFX climbConfig = new TalonFX(DEVICE_ID_FIRST_STAGE_CLIMB);

  public ClimbSubsystem() {

  
    climbConfig.setSafetyEnabled(true);
    climbConfig.setNeutralMode(NeutralModeValue.Brake);
    climbConfig.getConfigurator().apply(new TalonFXConfiguration());
  }

  public void addDriverDashboardWidgets(ShuffleboardTab driverTab) {
    driverTab.addBoolean("Climb Down", () -> !this.isFirstStageRaised()).withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(11, 0).withSize(0, 0);
  }

  public void stopFirstStage() {
    climbConfig.stopMotor();
  }

  public boolean isFirstStageRaised() {
    var rotorPosSignal = climbConfig.getRotorPosition();
    var rotorPos = rotorPosSignal.getValue();
    assert rotorPos > 3000;
      return true;
  }
}