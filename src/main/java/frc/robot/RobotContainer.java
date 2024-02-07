// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    populateSubsystemDashboard();
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
    controlBindings.intake().ifPresent(trigger -> trigger.toggleOnTrue(
        intakeSubsystem.deployAndRunIntakeCommand().alongWith(indexerSubsystem.intakeCommand())));
    
    controlBindings.intakeReverse().ifPresent(trigger -> trigger.whileTrue(
        intakeSubsystem.deployAndReverseIntakeCommand().alongWith(indexerSubsystem.unloadCommand())));
    
    // Elevator
    controlBindings.elevatorUp().ifPresent(trigger -> trigger.whileTrue(elevatorSubsystem.manualUpCommand()));
    controlBindings.elevatorUp().ifPresent(trigger -> trigger.whileTrue(elevatorSubsystem.manualDownCommand()));

    // Manual shoot - spin up for 1 second, then run indexer to shoot
    controlBindings.manualShoot().ifPresent(trigger -> trigger.whileTrue(
      shooterSubsystem.spinShooterAndAimCommand(RevolutionsPerSecond.of(50), Rotations.of(0.3))
        .alongWith(waitSeconds(1).andThen(indexerSubsystem.shootCommand()))
    ));

  }

  public void populateSysIdDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SysId");
    int columnIndex = 0;
    
    // Column 0 Drive
    tab.add("Drive Quasi Fwd", drivetrain.sysIdDriveQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Drive Quasi Rev", drivetrain.sysIdDriveQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Drive Dynam Fwd", drivetrain.sysIdDriveDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Drive Dynam Rev", drivetrain.sysIdDriveDynamCommand(kReverse)).withPosition(columnIndex, 3);
    tab.add("Slip Test", drivetrain.sysIdDriveSlipCommand()).withPosition(columnIndex, 4);

    // Column 1 Steer
    columnIndex++;
    tab.add("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 2 Rotation
    columnIndex++;
    tab.add("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 3 Intake
    columnIndex++;
    tab.add("Deploy Quasi Fwd", intakeSubsystem.sysIdDeployQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Deploy Quasi Rev", intakeSubsystem.sysIdDeployQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Roller Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Roller Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 4 Shooter
    columnIndex++;
    tab.add("Shoot Quasi Fwd", shooterSubsystem.sysIdShooterQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Shoot Quasi Rev", shooterSubsystem.sysIdShooterQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Shoot Dynam Fwd", shooterSubsystem.sysIdShooterDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Shoot Dynam Rev", shooterSubsystem.sysIdShooterDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 5 Shooter Aim
    columnIndex++;
    tab.add("Aim Quasi Fwd", shooterSubsystem.sysIdAimQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Aim Quasi Rev", shooterSubsystem.sysIdAimQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Aim Dynam Fwd", shooterSubsystem.sysIdAimDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Aim Dynam Rev", shooterSubsystem.sysIdAimDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 6 Indexer
    columnIndex++;
    tab.add("Index Quasi Fwd", indexerSubsystem.sysIdIndexerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Index Quasi Rev", indexerSubsystem.sysIdIndexerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Index Dynam Fwd", indexerSubsystem.sysIdIndexerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Index Dynam Rev", indexerSubsystem.sysIdIndexerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 7 Elevator
    columnIndex++;
    tab.add("Elev Quasi Fwd", elevatorSubsystem.sysIdQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Elev Quasi Rev", elevatorSubsystem.sysIdQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Elev Dynam Fwd", elevatorSubsystem.sysIdDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Elev Dynam Rev", elevatorSubsystem.sysIdDynamicCommand(kReverse)).withPosition(columnIndex, 3);

  }

  public void populateSubsystemDashboard() {
    var tab = Shuffleboard.getTab("Subsystems");

    // Intake grid
    var intakeGrid = tab.getLayout("Run Intake", BuiltInLayouts.kGrid)
        .withPosition(0, 0).withSize(2, 2)
        .withProperties(Map.of("Number of rows", 2, "Number of columns", 2));
        
    // Intake velocity
    var intakeVelocityWidget = intakeGrid.add("Velocity Intake", 0.0)
        .withPosition(0, 0);
    MutableMeasure<Velocity<Angle>> intakeVelocity = MutableMeasure.zero(RotationsPerSecond);

    var intakeVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = intakeVelocityWidget.getEntry().getDouble(0);
      intakeVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(intakeSubsystem.runIntakeRollersCommand(intakeVelocity)).withName("Intake Rollers");

    intakeGrid.add(intakeVelocityCommand).withPosition(1, 0);

    // Intake position
    var intakePositionWidget = intakeGrid.add("Position", 0.0)
        .withPosition(0, 1);
    MutableMeasure<Angle> intakePosition = MutableMeasure.zero(Rotations);

    var intakePositionCommand = Commands.run(() -> {
      var dashboardPosition = intakePositionWidget.getEntry().getDouble(0);
      intakePosition.mut_replace(dashboardPosition, Rotations);
    }).alongWith(intakeSubsystem.setIntakePosition(intakePosition)).withName("Intake Deploy");

    intakeGrid.add(intakePositionCommand).withPosition(1, 1);

    // Indexer grid
    var indexerGrid = tab.getLayout("Run Indexer", BuiltInLayouts.kGrid)
        .withPosition(2, 0).withSize(2, 1)
        .withProperties(Map.of("Number of rows", 1, "Number of columns", 2));

    // Indexer velocity
    var indexerVelocityWidget = indexerGrid.add("Velocity Indexer", 0.0)
        .withPosition(0, 0);
    MutableMeasure<Velocity<Angle>> indexerVelocity = MutableMeasure.zero(RotationsPerSecond);
    
    var indexerVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = indexerVelocityWidget.getEntry().getDouble(0);
      indexerVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(indexerSubsystem.runCommand(indexerVelocity)).withName("Indexer Velocity");

    indexerGrid.add(indexerVelocityCommand).withPosition(1, 0);

    // Shooter grid
    var shooterGrid = tab.getLayout("Run Shooter", BuiltInLayouts.kGrid)
        .withPosition(4, 0).withSize(2, 2)
        .withProperties(Map.of("Number of rows", 2, "Number of columns", 2));
    
    // Shooter velocity
    var shooterVelocityWidget = shooterGrid.add("Velocity Shooter", 0.0)
        .withPosition(0, 0);
    MutableMeasure<Velocity<Angle>> shooterVelocity = MutableMeasure.zero(RotationsPerSecond);
    
    var shooterVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = shooterVelocityWidget.getEntry().getDouble(0);
      shooterVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(shooterSubsystem.spinShooterCommand(shooterVelocity)).withName("Shooter Velocity");

    shooterGrid.add(shooterVelocityCommand).withPosition(1, 0);

    // Shooter Position
    var shooterPositoinWidget = shooterGrid.add("Position Shooter", 0.0)
        .withPosition(0, 1);
    MutableMeasure<Angle> shooterPosition = MutableMeasure.zero(Rotations);
    
    var shooterPositionCommand = Commands.run(() -> {
      var dashboardPosition = shooterPositoinWidget.getEntry().getDouble(0);
      shooterPosition.mut_replace(dashboardPosition, Rotations);
    }).alongWith(shooterSubsystem.setAimAngleCommand(shooterPosition)).withName("Shooter Position");

    shooterGrid.add(shooterPositionCommand).withPosition(1, 1);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
