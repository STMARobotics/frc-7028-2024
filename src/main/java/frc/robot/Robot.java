// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  @Logged
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private boolean hasPopulatedSysIdDashboard = false;
  private boolean hasPopulatedTestingDashboard = false;

  public Robot() {
    m_robotContainer = new RobotContainer();

    SignalLogger.start(); // CTRE logger
    DataLogManager.start(); // WPILib logger
    DriverStation.startDataLog(DataLogManager.getLog()); // Record both DS control and joystick data
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    DriverStation.getAlliance().ifPresent(alliance -> m_robotContainer.setAlliance(alliance));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DriverStation.getAlliance().ifPresent(alliance -> m_robotContainer.setAlliance(alliance));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    if (!hasPopulatedSysIdDashboard) {
      m_robotContainer.populateSysIdDashboard();
      hasPopulatedSysIdDashboard = true;
    }
    if (!hasPopulatedTestingDashboard) {
      m_robotContainer.populateTestingDashboard();
      hasPopulatedTestingDashboard = true;
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
