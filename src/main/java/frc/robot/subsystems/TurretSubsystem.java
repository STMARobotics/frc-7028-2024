package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET_ROT_CONTROL_FOLLOWER;
import static frc.robot.Constants.TurretConstants.DEVICE_ID_TURRET_ROT_CONTROL_LEADER;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TurretSubsystem extends SubsystemBase {
  private final MotionMagicVoltage motionalMagicControl = new MotionMagicVoltage(0).withEnableFOC(true);
  private final TalonFX rotLeaderMotor = new TalonFX(DEVICE_ID_TURRET_ROT_CONTROL_LEADER);
  private final TalonFX rotFollowerMotor = new TalonFX(DEVICE_ID_TURRET_ROT_CONTROL_FOLLOWER);
  private MotorController leaderMotorController;
  private MotorController followerMotorController;

  public void toAngle(double theta) {

  }

  public void stop() {
    
  }

}