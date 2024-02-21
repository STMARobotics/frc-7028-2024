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
import frc.robot.commands.ManualShootCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  private final ControlBindings controlBindings;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AmpShooterSubsystem ampShooterSubsystem = new AmpShooterSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final SendableChooser<Command> autoChooser;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY.in(MetersPerSecond));
  private final AutonomousBuilder autonomousBuilder = new AutonomousBuilder(
      shooterSubsystem, turretSubsystem, intakeSubsystem,
      ampShooterSubsystem, drivetrain, elevatorSubsystem);

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

    // drivetrain
    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.resetPose().ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative)));

    // shooter
    controlBindings.ManualShootCommand().ifPresent(trigger -> trigger.whileTrue(
        new ManualShootCommand(shooterSubsystem)));

  }

  public Command getAutonomousCommand() {
    return autonomousBuilder.getAutonomousCommand();
  }

  public void configureSysidDashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("sysid");
    tab.add("Shoot Quasi F", shooterSubsystem.sysIdShooterMotorQuasiCommand(kForward)).withPosition(0, 0);
    tab.add("Shoot Dynam F", shooterSubsystem.sysIdShooterMotorDynamCommand(kForward)).withPosition(0, 1);
    tab.add("Shoot Quasi R", shooterSubsystem.sysIdShooterMotorQuasiCommand(kReverse)).withPosition(0, 2);
    tab.add("Shoot Dynam R", shooterSubsystem.sysIdShooterMotorDynamCommand(kReverse)).withPosition(0, 3);

    tab.add("Alt Quasi F", shooterSubsystem.sysIdAltitudeMotorQuasiCommand(kForward)).withPosition(1, 0);
    tab.add("Alt Dynam F", shooterSubsystem.sysIdAltitudeMotorDynamCommand(kForward)).withPosition(1, 1);
    tab.add("Alt Quasi R", shooterSubsystem.sysIdAltitudeMotorQuasiCommand(kReverse)).withPosition(1, 2);
    tab.add("Alt Dynam R", shooterSubsystem.sysIdAltitudeMotorDynamCommand(kReverse)).withPosition(1, 3);

    tab.add("Shoot Intake Quasi F", shooterSubsystem.sysIdShooterIntakeMotorQuasiCommand(kForward)).withPosition(2, 0);
    tab.add("Shoot Intake Dynam F", shooterSubsystem.sysIdShooterIntakeMotorDynamCommand(kForward)).withPosition(2, 1);
    tab.add("Shoot Intake Quasi R", shooterSubsystem.sysIdShooterIntakeMotorQuasiCommand(kReverse)).withPosition(2, 2);
    tab.add("Shoot Intake Dynam R", shooterSubsystem.sysIdShooterIntakeMotorDynamCommand(kReverse)).withPosition(2, 3);

    tab.add("Rollers Quasi F", intakeSubsystem.intakeMotorQuasiCommand(kForward)).withPosition(3, 0);
    tab.add("Rollers Dynam F", intakeSubsystem.intakeMotorDynamCommand(kForward)).withPosition(3, 1);
    tab.add("Rollers Quasi R", intakeSubsystem.intakeMotorQuasiCommand(kReverse)).withPosition(3, 2);
    tab.add("Rollers Dynam R", intakeSubsystem.intakeMotorDynamCommand(kReverse)).withPosition(3, 3);

    tab.add("Lead Quasi F", elevatorSubsystem.sysIdElevatorMotorQuasiCommand(kForward)).withPosition(4, 0);
    tab.add("Lead Dynam F", elevatorSubsystem.sysIdElevatorMotorDynamCommand(kForward)).withPosition(4, 1);
    tab.add("Lead Quasi R", elevatorSubsystem.sysIdElevatorMotorQuasiCommand(kReverse)).withPosition(4, 2);
    tab.add("Lead Dynam R", elevatorSubsystem.sysIdElevatorMotorDynamCommand(kReverse)).withPosition(4, 3);

    tab.add("Amp Quasi F", ampShooterSubsystem.ampMotorQuasiCommand(kForward)).withPosition(6, 0);
    tab.add("Amp Dynam F", ampShooterSubsystem.ampMotorDynamCommand(kForward)).withPosition(6, 1);
    tab.add("Amp Quasi R", ampShooterSubsystem.ampMotorQuasiCommand(kReverse)).withPosition(6, 2);
    tab.add("Amp Dynam R", ampShooterSubsystem.ampMotorDynamCommand(kReverse)).withPosition(6, 3);

    tab.add("Drive Steer Quasi F", drivetrain.sysIdSteerQuasiCommand(kForward)).withPosition(4, 0);
    tab.add("Drive Steer Dynam F", drivetrain.sysIdSteerDynamCommand(kForward)).withPosition(4, 1);
    tab.add("Drive Steer Quasi R", drivetrain.sysIdSteerQuasiCommand(kReverse)).withPosition(4, 2);
    tab.add("Drive Steer Dynam R", drivetrain.sysIdSteerDynamCommand(kReverse)).withPosition(4, 3);

    tab.add("Drive Drive Quasi F", drivetrain.sysIdDriveQuasiCommand(kForward)).withPosition(5, 0);
    tab.add("Drive Drive Dynam F", drivetrain.sysIdDriveDynamCommand(kForward)).withPosition(5, 1);
    tab.add("Drive Drive Quasi R", drivetrain.sysIdDriveQuasiCommand(kReverse)).withPosition(5, 2);
    tab.add("Drive Drive Dynam R", drivetrain.sysIdDriveDynamCommand(kReverse)).withPosition(5, 3);

    tab.add("Drive Slip F", drivetrain.sysIdDriveSlipCommand()).withPosition(7, 0);
  }
}
