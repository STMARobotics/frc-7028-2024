// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Optional<Alliance> alliance = Optional.empty();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    SignalLogger.start();
  }

  @Override
  public void robotPeriodic() {
    checkDriverStationUpdate();
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
    checkDriverStationUpdate();
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
    checkDriverStationUpdate();
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

  /**
   * Checks the driverstation alliance. We have have to check repeatedly because we don't know when the
   * driverstation/FMS will connect, and the alliance can change at any time in the shop.
   */
  private void checkDriverStationUpdate() {
    // https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
    var currentAlliance = DriverStation.getAlliance();

    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance.isPresent() && !currentAlliance.equals(alliance)) {
      m_robotContainer.onAllianceChanged(currentAlliance.get());
      alliance = currentAlliance;
    }
  }
}
