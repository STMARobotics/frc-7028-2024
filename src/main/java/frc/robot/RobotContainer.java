// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final ControlBindings controlBindings;
  private final CommandSwerveDrivetrain driveTrain = TunerConstants.DriveTrain;
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY.in(MetersPerSecond));

  public RobotContainer() {
    configureSysidDashboard();
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0)) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);
    Shuffleboard.selectTab(driverTab.getTitle());

    driveTrain.getDaqThread().setThreadPriority(99);
    driveTrain.registerTelemetry(logger::telemeterize);

    configureDefaultCommands();
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    driveTrain.setDefaultCommand(new FieldOrientedDriveCommand(
        driveTrain,
        () -> driveTrain.getState().Pose.getRotation(),
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.omega()));
  }

  private void configureButtonBindings() {
    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(driveTrain.applyRequest(() -> brake)));
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(driveTrain.runOnce(driveTrain::seedFieldRelative)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureSysidDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("sysid");
    tab.add("Shoot Quasi F", shooterSubsystem.sysIdShooterMotorQuasiCommand(kForward));
    tab.add("Shoot Dynam F", shooterSubsystem.sysIdShooterMotorDynamCommand(kForward));
    tab.add("Shoot Quasi R", shooterSubsystem.sysIdShooterMotorQuasiCommand(kReverse));
    tab.add("Shoot Dynam R", shooterSubsystem.sysIdShooterMotorDynamCommand(kReverse));

    tab.add("Intake Roll Quasi F", intakeSubsystem.sysIdRollersMotorQuasiCommand(kForward));
    tab.add("Intake Roll Dynam F", intakeSubsystem.sysIdRollersMotorDynamCommand(kForward));
    tab.add("Intake Roll Quasi R", intakeSubsystem.sysIdRollersMotorQuasiCommand(kReverse));
    tab.add("Intake Roll Dynam R", intakeSubsystem.sysIdRollersMotorDynamCommand(kReverse));

    tab.add("Intake Deploy Quasi F", intakeSubsystem.sysIdDeployMotorQuasiCommand(kForward));
    tab.add("Intake Deploy Dynam F", intakeSubsystem.sysIdDeployMotorDynamCommand(kForward));
    tab.add("Intake Deploy Quasi R", intakeSubsystem.sysIdDeployMotorQuasiCommand(kReverse));
    tab.add("Intake Deploy Dynam R", intakeSubsystem.sysIdDeployMotorDynamCommand(kReverse));

    tab.add("Indexer Quasi F", indexerSubsystem.sysIdIndexerMotorQuasiCommand(kForward));
    tab.add("Indexer Dynam F", indexerSubsystem.sysIdIndexerMotorDynamCommand(kForward));
    tab.add("Indexer Quasi R", indexerSubsystem.sysIdIndexerMotorQuasiCommand(kReverse));
    tab.add("Indexer Dynam R", indexerSubsystem.sysIdIndexerMotorDynamCommand(kReverse));

    tab.add("Drive Steer Quasi F", driveTrain.sysIdSteerQuasiCommand(kForward));
    tab.add("Drive Steer Dynam F", driveTrain.sysIdSteerDynamCommand(kForward));
    tab.add("Drive Steer Quasi R", driveTrain.sysIdSteerQuasiCommand(kReverse));
    tab.add("Drive Steer Dynam R", driveTrain.sysIdSteerDynamCommand(kReverse));

    tab.add("Drive Drive Quasi F", driveTrain.sysIdDriveQuasiCommand(kForward));
    tab.add("Drive Drive Dynam F", driveTrain.sysIdDriveDynamCommand(kForward));
    tab.add("Drive Drive Quasi R", driveTrain.sysIdDriveQuasiCommand(kReverse));
    tab.add("Drive Drive Dynam R", driveTrain.sysIdDriveDynamCommand(kReverse));

    tab.add("Elevator Quasi F", elevatorSubsystem.sysIdElevatorQuasiCommand(kForward));
    tab.add("Elevator Dynam F", elevatorSubsystem.sysIdElevatorDynamCommand(kForward));
    tab.add("Elevator Quasi R", elevatorSubsystem.sysIdElevatorQuasiCommand(kReverse));
    tab.add("Elevator Dynam R", elevatorSubsystem.sysIdElevatorDynamCommand(kReverse));

  }
}