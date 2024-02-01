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

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY.in(MetersPerSecond));

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
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
    populateSysIdDashboard();
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
    // Driving
    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));

    // Reset field relative heading
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative)));

    // Intake
    controlBindings.intake().ifPresent(trigger -> trigger.onTrue(intakeSubsystem.deployAndRunIntakeCommand()));
    controlBindings.retractIntake().ifPresent(trigger -> trigger.onTrue(intakeSubsystem.retractIntakeCommand()));
  }

  public void populateSysIdDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SysId");
    
    // Column 0 Drive
    tab.add("Drive Quasi Fwd", drivetrain.sysIdDriveQuasiCommand(kForward)).withPosition(0, 0);
    tab.add("Drive Quasi Rev", drivetrain.sysIdDriveQuasiCommand(kReverse)).withPosition(0, 1);
    tab.add("Drive Dynam Fwd", drivetrain.sysIdDriveDynamCommand(kForward)).withPosition(0, 2);
    tab.add("Drive Dynam Rev", drivetrain.sysIdDriveDynamCommand(kReverse)).withPosition(0, 3);
    tab.add("Slip Test", drivetrain.sysIdDriveSlipCommand()).withPosition(0, 4);

    // Column 1 Steer
    tab.add("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward)).withPosition(1, 0);
    tab.add("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse)).withPosition(1, 1);
    tab.add("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward)).withPosition(1, 2);
    tab.add("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse)).withPosition(1, 3);

    // Column 2 Intake
    tab.add("Deploy Quasi Fwd", intakeSubsystem.sysIdDeployQuasistaticCommand(kForward)).withPosition(2, 0);
    tab.add("Deploy Quasi Rev", intakeSubsystem.sysIdDeployQuasistaticCommand(kReverse)).withPosition(2, 1);
    tab.add("Roller Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(2, 2);
    tab.add("Roller Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(2, 3);

    // Column 3 Shooter
    tab.add("Shoot Quasi Fwd", shooterSubsystem.sysIdShooterQuasistaticCommand(kForward)).withPosition(3, 0);
    tab.add("Shoot Quasi Rev", shooterSubsystem.sysIdShooterQuasistaticCommand(kReverse)).withPosition(3, 1);
    tab.add("Shoot Dynam Fwd", shooterSubsystem.sysIdShooterDyanmicCommand(kForward)).withPosition(3, 2);
    tab.add("Shoot Dynam Rev", shooterSubsystem.sysIdShooterDyanmicCommand(kReverse)).withPosition(3, 3);

    //Column 4 Indexer
    tab.add("Index Quasi Fwd", indexerSubsystem.sysIdIndexerQuasistaticCommand(kForward)).withPosition(4, 0);
    tab.add("Index Quasi Rev", indexerSubsystem.sysIdIndexerQuasistaticCommand(kReverse)).withPosition(4, 1);
    tab.add("Index Dynam Fwd", indexerSubsystem.sysIdIndexerDynamicCommand(kForward)).withPosition(4, 2);
    tab.add("Index Dynam Rev", indexerSubsystem.sysIdIndexerDynamicCommand(kReverse)).withPosition(4, 3);

    //Column 5 Elevator
    tab.add("Elevator Quasi Fwd", elevatorSubsystem.sysIdQuasistaticCommand(kForward)).withPosition(5, 0);
    tab.add("Elevator Quasi Rev", elevatorSubsystem.sysIdQuasistaticCommand(kReverse)).withPosition(5, 1);
    tab.add("Elevator Dynam Fwd", elevatorSubsystem.sysIdDynamicCommand(kForward)).withPosition(5, 2);
    tab.add("Elevator Dynam Rev", elevatorSubsystem.sysIdDynamicCommand(kReverse)).withPosition(5, 3);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
