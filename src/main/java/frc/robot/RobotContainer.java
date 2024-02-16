// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.telemetry.DriverTelemetry;
import frc.robot.telemetry.DrivetrainTelemetry;

public class RobotContainer {

  private final ControlBindings controlBindings;

  private final DriverTelemetry driverTelemetry = new DriverTelemetry();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry(MAX_VELOCITY.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AmperSubsystem amperSubsystem = new AmperSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem(driverTelemetry::telemeterizeIndexer);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(driverTelemetry::telemeterizeElevator);

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);

    drivetrain.getDaqThread().setThreadPriority(99);
    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

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

    // Manual shoot - spin up, then run indexer to shoot
    controlBindings.manualShoot().ifPresent(trigger -> trigger.whileTrue(
      shooterSubsystem.spinShooterCommand(RevolutionsPerSecond.of(40))
        .alongWith(waitUntil(shooterSubsystem::isReadyToShoot).andThen(indexerSubsystem.shootCommand()))
    ));

    controlBindings.shoot().ifPresent(trigger -> trigger.whileTrue(
      new ShootCommand(drivetrain, indexerSubsystem, shooterSubsystem)));

  }

  public void populateSysIdDashboard() {
    var tab = Shuffleboard.getTab("Drive SysId");
    int columnIndex = 0;
    
    // Column 0 Drive
    tab.add("Drive Quasi Fwd", drivetrain.sysIdDriveQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Drive Quasi Rev", drivetrain.sysIdDriveQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Drive Dynam Fwd", drivetrain.sysIdDriveDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Drive Dynam Rev", drivetrain.sysIdDriveDynamCommand(kReverse)).withPosition(columnIndex, 3);
    tab.add("Slip Test", drivetrain.sysIdDriveSlipCommand()).withPosition(columnIndex, 4);

    // Column 2 Steer
    columnIndex += 2;
    tab.add("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 4 Rotation
    columnIndex += 2;
    tab.add("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse)).withPosition(columnIndex, 3);

    tab = Shuffleboard.getTab("Sub SysId");
    // Column 0 Intake
    columnIndex = 0;
    tab.add("Intake Quasi Fwd", intakeSubsystem.sysIdRollerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Intake Quasi Rev", intakeSubsystem.sysIdRollerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Intake Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Intake Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 2 Shooter
    columnIndex += 2;
    tab.add("Shoot Quasi Fwd", shooterSubsystem.sysIdShooterQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Shoot Quasi Rev", shooterSubsystem.sysIdShooterQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Shoot Dynam Fwd", shooterSubsystem.sysIdShooterDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Shoot Dynam Rev", shooterSubsystem.sysIdShooterDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 4 Shooter Aim
    columnIndex += 2;
    tab.add("Aim Quasi Fwd", shooterSubsystem.sysIdAimQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Aim Quasi Rev", shooterSubsystem.sysIdAimQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Aim Dynam Fwd", shooterSubsystem.sysIdAimDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Aim Dynam Rev", shooterSubsystem.sysIdAimDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 6 Indexer
    columnIndex += 2;
    tab.add("Index Quasi Fwd", indexerSubsystem.sysIdIndexerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Index Quasi Rev", indexerSubsystem.sysIdIndexerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Index Dynam Fwd", indexerSubsystem.sysIdIndexerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Index Dynam Rev", indexerSubsystem.sysIdIndexerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 8 Elevator
    columnIndex += 2;
    tab.add("Elev Quasi Fwd", elevatorSubsystem.sysIdQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Elev Quasi Rev", elevatorSubsystem.sysIdQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Elev Dynam Fwd", elevatorSubsystem.sysIdDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Elev Dynam Rev", elevatorSubsystem.sysIdDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 10 Intake
    columnIndex += 2;
    tab.add("Amper Quasi Fwd", amperSubsystem.sysIdRollerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Amper Quasi Rev", amperSubsystem.sysIdRollerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Amper Dynam Fwd", amperSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Amper Dynam Rev", amperSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

  }

  public void setAlliance(Alliance alliance) {
    drivetrain.setOperatorPerspectiveForward(
        alliance == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
