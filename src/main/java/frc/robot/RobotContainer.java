// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(driverTelemetry::telemeterizeIntake);
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
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(() -> 
        drivetrain.seedFieldRelative(new Pose2d(1.5,  Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0))))));

    // Intake
    controlBindings.intake().ifPresent(trigger -> trigger.toggleOnTrue(
        intakeSubsystem.deployAndRunIntakeCommand().alongWith(indexerSubsystem.intakeCommand())
            .until(indexerSubsystem::isFullSensorTripped)));
    
    controlBindings.intakeRetract().ifPresent(trigger -> trigger.onTrue(
        intakeSubsystem.retractIntakeCommand().alongWith(indexerSubsystem.stopCommand())));

    controlBindings.intakeReverse().ifPresent(trigger -> trigger.whileTrue(
        intakeSubsystem.deployAndReverseIntakeCommand().alongWith(
          waitUntil(intakeSubsystem::isDeployed).andThen(indexerSubsystem.unloadCommand()))));
    
    // Elevator
    controlBindings.elevatorUp().ifPresent(trigger -> trigger.whileTrue(elevatorSubsystem.manualUpCommand()));
    controlBindings.elevatorDown().ifPresent(trigger -> trigger.whileTrue(elevatorSubsystem.manualDownCommand()));

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
    tab.add("Deploy Quasi Fwd", intakeSubsystem.sysIdDeployQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Deploy Quasi Rev", intakeSubsystem.sysIdDeployQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Deploy Dynam Fwd", intakeSubsystem.sysIdDeployDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Deploy Dynam Rev", intakeSubsystem.sysIdDeployDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 2 Intake
    columnIndex += 2;
    tab.add("Roller Quasi Fwd", intakeSubsystem.sysIdRollerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Roller Quasi Rev", intakeSubsystem.sysIdRollerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Roller Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Roller Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 4 Shooter
    columnIndex += 2;
    tab.add("Shoot Quasi Fwd", shooterSubsystem.sysIdShooterQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Shoot Quasi Rev", shooterSubsystem.sysIdShooterQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Shoot Dynam Fwd", shooterSubsystem.sysIdShooterDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Shoot Dynam Rev", shooterSubsystem.sysIdShooterDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Column 6 Shooter Aim
    columnIndex += 2;
    tab.add("Aim Quasi Fwd", shooterSubsystem.sysIdAimQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Aim Quasi Rev", shooterSubsystem.sysIdAimQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Aim Dynam Fwd", shooterSubsystem.sysIdAimDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Aim Dynam Rev", shooterSubsystem.sysIdAimDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 8 Indexer
    columnIndex += 2;
    tab.add("Index Quasi Fwd", indexerSubsystem.sysIdIndexerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Index Quasi Rev", indexerSubsystem.sysIdIndexerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Index Dynam Fwd", indexerSubsystem.sysIdIndexerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Index Dynam Rev", indexerSubsystem.sysIdIndexerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    //Column 10 Elevator
    columnIndex += 2;
    tab.add("Elev Quasi Fwd", elevatorSubsystem.sysIdQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Elev Quasi Rev", elevatorSubsystem.sysIdQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Elev Dynam Fwd", elevatorSubsystem.sysIdDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Elev Dynam Rev", elevatorSubsystem.sysIdDynamicCommand(kReverse)).withPosition(columnIndex, 3);

  }

  public void populateSubsystemDashboard() {
    var tab = Shuffleboard.getTab("Subsystems");

    // Intake grid
    var intakeList = tab.getLayout("Run Intake", BuiltInLayouts.kList)
        .withPosition(0, 0).withSize(2, 3);
        
    // Intake velocity
    var intakeVelocityWidget = intakeList.add("Set Roller Velocity", 0.0);
    MutableMeasure<Velocity<Angle>> intakeVelocity = MutableMeasure.zero(RotationsPerSecond);

    var intakeVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = intakeVelocityWidget.getEntry().getDouble(0);
      intakeVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(intakeSubsystem.runIntakeRollersCommand(intakeVelocity)).withName("Intake Rollers");

    intakeList.add(intakeVelocityCommand);

    // Intake position
    var intakePositionWidget = intakeList.add("Set Deploy Position", 0.0);
    MutableMeasure<Angle> intakePosition = MutableMeasure.zero(Rotations);

    var intakePositionCommand = Commands.run(() -> {
      var dashboardPosition = intakePositionWidget.getEntry().getDouble(0);
      intakePosition.mut_replace(dashboardPosition, Rotations);
    }).alongWith(intakeSubsystem.setIntakePositionCommand(intakePosition)).withName("Intake Deploy");

    intakeList.add(intakePositionCommand);

    // Indexer grid
    var indexerList = tab.getLayout("Run Indexer", BuiltInLayouts.kList)
        .withPosition(2, 0).withSize(2, 6);

    // Indexer velocity
    var indexerVelocityWidget = indexerList.add("Set Velocity", 0.0).withPosition(0, 0);
    MutableMeasure<Velocity<Angle>> indexerVelocity = MutableMeasure.zero(RotationsPerSecond);
    
    var indexerVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = indexerVelocityWidget.getEntry().getDouble(0);
      indexerVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(indexerSubsystem.runCommand(indexerVelocity)).withName("Indexer");

    indexerList.add(indexerVelocityCommand).withPosition(0, 1);

    // Indexer current velocity
    indexerList.addDouble("Left Velocity", () -> indexerSubsystem.getLeftVelocity().in(RotationsPerSecond))
        .withWidget(BuiltInWidgets.kGraph).withPosition(0, 2);
    indexerList.addDouble("Right Velocity", () -> indexerSubsystem.getRightVelocity().in(RotationsPerSecond))
        .withWidget(BuiltInWidgets.kGraph).withPosition(0, 3);

    // Shooter grid
    var shooterList = tab.getLayout("Run Shooter", BuiltInLayouts.kList)
        .withPosition(4, 0).withSize(2, 3);
    
    // Shooter velocity
    var shooterVelocityWidget = shooterList.add("Set Velocity", 0.0);
    MutableMeasure<Velocity<Angle>> shooterVelocity = MutableMeasure.zero(RotationsPerSecond);
    
    var shooterVelocityCommand = Commands.run(() -> {
      var dashboardVelocity = shooterVelocityWidget.getEntry().getDouble(0);
      shooterVelocity.mut_replace(dashboardVelocity, RotationsPerSecond);
    }).alongWith(shooterSubsystem.spinShooterCommand(shooterVelocity)).withName("Shooter Velocity");

    shooterList.add(shooterVelocityCommand);

    // Shooter wheel rotation / position
    var shooterPositoinWidget = shooterList.add("Set Rotations", 0.0);
    MutableMeasure<Angle> shooterRotations = MutableMeasure.zero(Rotations);
    
    var shooterPositionCommand = Commands.run(() -> {
      var dashboardRotations = shooterPositoinWidget.getEntry().getDouble(0);
      shooterRotations.mut_replace(dashboardRotations, Rotations);
    }).alongWith(shooterSubsystem.rotateShooterCommand(shooterRotations)).withName("Shooter Rotation");

    shooterList.add(shooterPositionCommand);
    
    // Shooter aim grid
    var shooterAimList = tab.getLayout("Aim Shooter", BuiltInLayouts.kList)
        .withPosition(6, 0).withSize(2, 3);

    // Aim Shooter
    var shooterAimWidget = shooterAimList.add("Set Position", 0.0);
    MutableMeasure<Angle> shooterAimPosition = MutableMeasure.zero(Rotations);
    
    var shooterAimCommand = Commands.run(() -> {
      var dashboardPosition = shooterAimWidget.getEntry().getDouble(0);
      shooterAimPosition.mut_replace(dashboardPosition, Rotations);
    }).alongWith(shooterSubsystem.setAimAngleCommand(shooterAimPosition)).withName("Aim Position");

    shooterAimList.add(shooterAimCommand);

    // Aim Voltage
    var shooterAimVoltageWidget = shooterAimList.add("Set Voltage", 0.0);
    MutableMeasure<Voltage> shooterAimVoltage = MutableMeasure.zero(Volts);
    
    var shooterAimVoltageCommand = Commands.run(() -> {
      var dashboardVoltage = shooterAimVoltageWidget.getEntry().getDouble(0);
      shooterAimVoltage.mut_replace(dashboardVoltage, Volts);
    }).alongWith(shooterSubsystem.setAimVoltageCommand(shooterAimVoltage)).withName("Aim Voltage");

    shooterAimList.add(shooterAimVoltageCommand);

    // Elevator
    var elevatorList = tab.getLayout("Elevator", BuiltInLayouts.kList)
        .withPosition(8, 0).withSize(2, 3);

    var elevatorWidget = elevatorList.add("Set Voltage", 0.0);
    MutableMeasure<Voltage> elevatorVoltage = MutableMeasure.zero(Volts);
    
    var elevatorVoltageCommand = Commands.run(() -> {
      var dashboardVoltage = elevatorWidget.getEntry().getDouble(0);
      elevatorVoltage.mut_replace(dashboardVoltage, Volts);
    }).alongWith(elevatorSubsystem.elevatorVoltageCommand(elevatorVoltage)).withName("Elevator Voltage");

    elevatorList.add(elevatorVoltageCommand);

  }

  public void setAlliance(Alliance alliance) {
    drivetrain.setOperatorPerspectiveForward(
        alliance == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
