// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.BIND_SYS_ID;
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
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  private final ControlBindings controlBindings;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

    // SysId tests
    if (BIND_SYS_ID) {
      controlBindings.sysIdDriveForwardQuasiTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runDriveQuasiTest(kForward)));
      controlBindings.sysIdDriveReverseQuasiTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runDriveQuasiTest(kReverse)));

      controlBindings.sysIdDriveForwardDynamTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runDriveDynamTest(kForward)));
      controlBindings.sysIdDriveReverseDynamTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runDriveDynamTest(kReverse)));

      controlBindings.sysIdSteerForwardQuasiTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runSteerQuasiTest(kForward)));
      controlBindings.sysIdSteerReverseQuasiTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runSteerQuasiTest(kReverse)));

      controlBindings.sysIdSteerForwardDynamTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runSteerDynamTest(kForward)));
      controlBindings.sysIdSteerReverseDynamTest()
          .ifPresent(trigger -> trigger.whileTrue(drivetrain.runSteerDynamTest(kReverse)));

      controlBindings.sysIdDriveSlipTest().ifPresent(trigger -> trigger.whileTrue(drivetrain.runDriveSlipTest()));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
