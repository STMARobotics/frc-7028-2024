package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRotations;
import static edu.wpi.first.math.util.Units.rotationsToDegrees;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET_ROT_CONTROL_LEADER;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(DEVICE_ID_TURRET_ROT_CONTROL_LEADER);
  private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
  private StatusSignal<Double> motorRotations = turretMotor.getPosition();


  public double getCurrentAngle() {
    return rotationsToDegrees(motorRotations.getValueAsDouble());
  }

  public void toAngle(Rotation2d angle) {
    turretMotor.setControl(
        motionMagicControl.withPosition(degreesToRotations(angle.getDegrees())));
  }

  public void stop() {
    turretMotor.stopMotor();
  }

}