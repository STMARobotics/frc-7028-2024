// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class elevatorSubsystem extends TimedRobot {
  private final TalonFX m_fx = new TalonFX(1, "canivore");
  private final TalonFX m_fx2 = new TalonFX(1, "canivore");
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final XboxController m_joystick = new XboxController(0);

  private int m_printCount = 0;

  // fix line 34
  //private final Mechanism m_mechanisms = new Mechanism(null, null, null);

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
    PhysicsSim.getInstance().addTalonFX(m_fx2, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(cfg);
      status = m_fx2.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void robotPeriodic() {
    if (m_printCount++ > 10) {
      m_printCount = 0;
      System.out.println("Pos: " + m_fx.getPosition());
      System.out.println("Vel: " + m_fx.getVelocity());
      System.out.println();
      System.out.println("Pos: " + m_fx2.getPosition());
      System.out.println("Vel: " + m_fx2.getVelocity());
    }
    // fix line 90
    //m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Deadband the joystick */
    double leftY = m_joystick.getLeftY();
    if(leftY > -0.1 && leftY < 0.1) leftY = 0;

    m_fx.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0));
    if(m_joystick.getBButton()) {
      m_fx.setPosition(1);
    m_fx2.setControl(m_mmReq.withPosition(leftY * 10).withSlot(1));
    if(m_joystick.getAButton()) {
      m_fx2.setPosition(1);
    }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
