// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {

  private final ControlBindings controlBindings;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystem");
  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY.in(MetersPerSecond));

  private IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0)) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);
    Shuffleboard.selectTab(driverTab.getTitle());

    drivetrain.getDaqThread().setThreadPriority(99);
    drivetrain.registerTelemetry(logger::telemeterize);

    configureDefaultCommands();
    configureButtonBindings();

    configureSmartDashboard();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(
      drivetrain,
      () -> drivetrain.getState().Pose.getRotation(),
      controlBindings.translationX(),
      controlBindings.translationY(),
      controlBindings.omega()));
  }

  private void configureButtonBindings() {
    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /*
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/organizing-widgets.html#adding-widgets-to-layouts   
   * 2020 IndexerSubsystem.java: https://github.com/STMARobotics/frc-7028-2020/blob/b4bafc1792f9b4ddc187f8470b0808450ce86903/src/main/java/frc/robot/subsystems/IndexerSubsystem.java#L90C3-L98C4 
   * 2020 RobotContainer.java:   https://github.com/STMARobotics/frc-7028-2020/blob/b4bafc1792f9b4ddc187f8470b0808450ce86903/src/main/java/frc/robot/RobotContainer.java#L240C3-L300C4 
   */
  private void configureSmartDashboard() {
    configureSubsystemTab();
    configureDriverTab();
  }

  private void configureSubsystemTab() {
    ShuffleboardLayout indexerSubsystemLayout = subsystemTab.getLayout("Indexer", BuiltInLayouts.kList);
    indexerSubsystem.addSubsystemDashboardWidgets(indexerSubsystemLayout);
  }

  private void configureDriverTab() {
    indexerSubsystem.addDriverDashboardWidgets(driverTab);
  }
}
