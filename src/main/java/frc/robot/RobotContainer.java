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
  private final ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystem");
  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY.in(MetersPerSecond));

  private IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

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
    
    configureSmartDashboard();
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
  
  /*
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/organizing-widgets.html#adding-widgets-to-layouts   
   * 2020 IndexerSubsystem.java: https://github.com/STMARobotics/frc-7028-2020/blob/b4bafc1792f9b4ddc187f8470b0808450ce86903/src/main/java/frc/robot/subsystems/IndexerSubsystem.java#L90C3-L98C4 
   * 2020 RobotContainer.java:   https://github.com/STMARobotics/frc-7028-2020/blob/b4bafc1792f9b4ddc187f8470b0808450ce86903/src/main/java/frc/robot/RobotContainer.java#L240C3-L300C4 
   */
  private void configureSmartDashboard() {
    configureSubsystemTab();
    configureDriverTab();
    configureSysidDashboard();
  }

  private void configureSubsystemTab() {
    ShuffleboardLayout indexerSubsystemLayout = subsystemTab.getLayout("Indexer", BuiltInLayouts.kList);
    indexerSubsystem.addSubsystemDashboardWidgets(indexerSubsystemLayout);
  }

  private void configureDriverTab() {
    indexerSubsystem.addDriverDashboardWidgets(driverTab);
  }
}
 public void configureSysidDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("sysid");
    tab.add("Shoot Quasi F", shooterSubsystem.sysIdShooterMotorQuasiCommand(kForward)).withPosition(0, 0);
    tab.add("Shoot Dynam F", shooterSubsystem.sysIdShooterMotorDynamCommand(kForward)).withPosition(0, 1);
    tab.add("Shoot Quasi R", shooterSubsystem.sysIdShooterMotorQuasiCommand(kReverse)).withPosition(0, 2);
    tab.add("Shoot Dynam R", shooterSubsystem.sysIdShooterMotorDynamCommand(kReverse)).withPosition(0, 3);

    tab.add("Intake Roll Quasi F", intakeSubsystem.sysIdRollersMotorQuasiCommand(kForward)).withPosition(1, 0);
    tab.add("Intake Roll Dynam F", intakeSubsystem.sysIdRollersMotorDynamCommand(kForward)).withPosition(1, 1);
    tab.add("Intake Roll Quasi R", intakeSubsystem.sysIdRollersMotorQuasiCommand(kReverse)).withPosition(1, 2);
    tab.add("Intake Roll Dynam R", intakeSubsystem.sysIdRollersMotorDynamCommand(kReverse)).withPosition(1, 3);

    tab.add("Intake Deploy Quasi F", intakeSubsystem.sysIdDeployMotorQuasiCommand(kForward)).withPosition(2, 0);
    tab.add("Intake Deploy Dynam F", intakeSubsystem.sysIdDeployMotorDynamCommand(kForward)).withPosition(2, 1);
    tab.add("Intake Deploy Quasi R", intakeSubsystem.sysIdDeployMotorQuasiCommand(kReverse)).withPosition(2, 2);
    tab.add("Intake Deploy Dynam R", intakeSubsystem.sysIdDeployMotorDynamCommand(kReverse)).withPosition(2, 3);

    tab.add("Indexer Quasi F", indexerSubsystem.sysIdIndexerMotorQuasiCommand(kForward)).withPosition(3, 0);
    tab.add("Indexer Dynam F", indexerSubsystem.sysIdIndexerMotorDynamCommand(kForward)).withPosition(3, 1);
    tab.add("Indexer Quasi R", indexerSubsystem.sysIdIndexerMotorQuasiCommand(kReverse)).withPosition(3, 2);
    tab.add("Indexer Dynam R", indexerSubsystem.sysIdIndexerMotorDynamCommand(kReverse)).withPosition(3, 3);

    tab.add("Drive Steer Quasi F", driveTrain.sysIdSteerQuasiCommand(kForward)).withPosition(4, 0);
    tab.add("Drive Steer Dynam F", driveTrain.sysIdSteerDynamCommand(kForward)).withPosition(4, 1);
    tab.add("Drive Steer Quasi R", driveTrain.sysIdSteerQuasiCommand(kReverse)).withPosition(4, 2);
    tab.add("Drive Steer Dynam R", driveTrain.sysIdSteerDynamCommand(kReverse)).withPosition(4, 3);

    tab.add("Drive Drive Quasi F", driveTrain.sysIdDriveQuasiCommand(kForward)).withPosition(5, 0);
    tab.add("Drive Drive Dynam F", driveTrain.sysIdDriveDynamCommand(kForward)).withPosition(5, 1);
    tab.add("Drive Drive Quasi R", driveTrain.sysIdDriveQuasiCommand(kReverse)).withPosition(5, 2);
    tab.add("Drive Drive Dynam R", driveTrain.sysIdDriveDynamCommand(kReverse)).withPosition(5, 3);

    tab.add("Drive Slip F", driveTrain.sysIdDriveSlipCommand()).withPosition(7, 0);

    tab.add("Elevator Quasi F", elevatorSubsystem.sysIdElevatorQuasiCommand(kForward)).withPosition(6, 0);
    tab.add("Elevator Dynam F", elevatorSubsystem.sysIdElevatorDynamCommand(kForward)).withPosition(6, 1);
    tab.add("Elevator Quasi R", elevatorSubsystem.sysIdElevatorQuasiCommand(kReverse)).withPosition(6, 2);
    tab.add("Elevator Dynam R", elevatorSubsystem.sysIdElevatorDynamCommand(kReverse)).withPosition(6, 3);
  }
}
