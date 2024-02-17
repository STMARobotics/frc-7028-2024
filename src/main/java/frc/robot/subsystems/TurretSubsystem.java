package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.rotationsToDegrees;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(DEVICE_ID_TURRET);
  private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
  private StatusSignal<Double> motorRotations = turretMotor.getPosition();

  public TurretSubsystem() {
    TalonFXConfiguration turretMotorConfiguration = new TalonFXConfiguration();
    turretMotor.getConfigurator().apply(turretMotorConfiguration);
  
  }

  public Rotation2d getCurrentAngle() {
    return new Rotation2d(Degrees.of(rotationsToDegrees(motorRotations.getValueAsDouble())));
  }

  public void toAngle(Rotation2d angle) {
    turretMotor.setControl(
        motionMagicControl.withPosition(angle.getRotations()));
  }

  public void stop() {
    turretMotor.stopMotor();
  }

}