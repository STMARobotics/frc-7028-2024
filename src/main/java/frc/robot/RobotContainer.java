// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY;
import static frc.robot.Constants.LEDConstants.NOTE_COLOR;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_MID_BLUE;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_MID_RED;
import static frc.robot.Constants.TurretConstants.INTAKE_YAW;
import static frc.robot.Constants.TurretConstants.PITCH_LIMIT_FORWARD;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BabyBirdCommand;
import frc.robot.commands.DefaultTurretCommand;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ShootTeleopCommand;
import frc.robot.commands.SpeakerOrBlinkCommand;
import frc.robot.commands.StockpileOrBlinkCommand;
import frc.robot.commands.TuneShootingCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBlinkCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.commands.testing.LEDProgressBarCommand;
import frc.robot.commands.testing.TestCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.DemoControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  private static final boolean DEMO_MODE = false;

  private final ControlBindings controlBindings;

  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry(MAX_VELOCITY.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final TestCommand testCommand = new TestCommand(intakeSubsystem, shooterSubsystem, turretSubsystem);

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final SendableChooser<Command> autoChooser;
  private final Field2d field2d = new Field2d();

  private final AutoCommands autoCommands = new AutoCommands(
      drivetrain, shooterSubsystem, turretSubsystem, intakeSubsystem, ledSubsystem);

  public RobotContainer() {
    // Configure control binding scheme
    if (DEMO_MODE) {
      controlBindings = new DemoControlBindings();
    } else if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    autoCommands.registerPPNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();

    drivetrain.getDaqThread().setThreadPriority(99);
    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    configureDefaultCommands();
    configureButtonBindings();
    configureDashboard();

    new LEDBootAnimationCommand(ledSubsystem).schedule();

    // Warm up PathPlanner https://pathplanner.dev/pplib-follow-a-single-path.html#java-warmup
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(
        drivetrain,
        controlBindings.translationX(),
        controlBindings.translationY(),
        controlBindings.omega()));

    ledSubsystem.setDefaultCommand(new DefaultLEDCommand(ledSubsystem, turretSubsystem::hasNote));

    turretSubsystem.setDefaultCommand(new DefaultTurretCommand(turretSubsystem));
  }

  private void configureDashboard() {
    // Auto selector
    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);

    // Driver camera
    driverTab.add(new HttpCamera("photonvision_Port_1182_Output_MJPEG_Server", "http://10.70.28.11:1182"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", true, "showControls", false))
        .withSize(4, 4).withPosition(2, 0);

    // Note sensor
    driverTab.addBoolean("Turret", turretSubsystem::hasNote)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 0);

    // Turret
    driverTab.addNumber("Pitch", () -> turretSubsystem.getPitch().in(Degrees))
        .withWidget(BuiltInWidgets.kTextView).withPosition(6, 1);

    driverTab.addNumber("Yaw", () -> turretSubsystem.getYaw().in(Degrees))
        .withWidget(BuiltInWidgets.kTextView).withPosition(6, 2);

    // Pose estimation
    driverTab.add(field2d).withPosition(7, 0).withSize(4, 2).withPosition(7, 0);
    driverTab.addString("Pose", () -> {
      var pose = drivetrain.getState().Pose;
      if (pose == null) {
        pose = new Pose2d();
      }
      field2d.setRobotPose(pose);
      return String.format("(%.3f, %.3f) %.2f deg",
          pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }).withSize(2, 1).withPosition(7, 2);

  }

  private void configureButtonBindings() {
    // Driving
    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));

    // Intake
    controlBindings.intake().ifPresent(trigger -> trigger.onTrue(autoCommands.intakeToTurret()));

    controlBindings.intakeStop().ifPresent(trigger -> trigger.onTrue(runOnce(() -> {
      intakeSubsystem.stop();
      turretSubsystem.stop();
    }, intakeSubsystem, turretSubsystem)));

    controlBindings.eject().ifPresent(trigger -> trigger.whileTrue(
        new EjectCommand(intakeSubsystem, turretSubsystem, shooterSubsystem, drivetrain)));

    controlBindings.babyBird().ifPresent(trigger -> trigger.whileTrue(
        new BabyBirdCommand(turretSubsystem, shooterSubsystem)
            .deadlineWith(new LEDBlinkCommand(ledSubsystem, NOTE_COLOR, 0.1))));

    // Speaker
    controlBindings.scoreSpeaker().ifPresent(trigger -> trigger.whileTrue(
        new SpeakerOrBlinkCommand(drivetrain, shooterSubsystem, turretSubsystem, ledSubsystem,
            controlBindings.translationX(), controlBindings.translationY(), controlBindings.omega())));

    controlBindings.manualShoot().ifPresent(trigger -> trigger.whileTrue(new ManualShootCommand(
        turretSubsystem, shooterSubsystem, RotationsPerSecond.of(50), Degrees.of(35), Degrees.of(180))));

    // Amp
    controlBindings.scoreAmp().ifPresent(trigger -> trigger.whileTrue(autoCommands.scoreAmp()));

    controlBindings.stockpile().ifPresent(trigger -> trigger.whileTrue(new StockpileOrBlinkCommand(
        drivetrain, shooterSubsystem, turretSubsystem, ledSubsystem, controlBindings.translationX(),
        controlBindings.translationY(), controlBindings.omega())));
    
    controlBindings.stockpileMiddle().ifPresent(trigger -> trigger.whileTrue(new ShootTeleopCommand(
        drivetrain, shooterSubsystem, turretSubsystem, ledSubsystem, controlBindings.translationX(),
        controlBindings.translationY(), STOCKPILE_MID_RED, STOCKPILE_MID_BLUE, STOCKPILE_INTERPOLATOR, 0.3)));
    
    // Misc
    controlBindings.liftShooter().ifPresent(trigger -> trigger.whileTrue(turretSubsystem.run(() -> {
      turretSubsystem.moveToPitchPosition(PITCH_LIMIT_FORWARD);
      turretSubsystem.moveToYawPosition(INTAKE_YAW);
    })));

    // Testing
    controlBindings.tuneShooting().ifPresent(trigger -> trigger.whileTrue(
        new TuneShootingCommand(turretSubsystem, shooterSubsystem, ledSubsystem, () -> drivetrain.getState().Pose)));

    // Demo shots
    controlBindings.demoToss1().ifPresent(trigger -> trigger.whileTrue(
        new ManualShootCommand(turretSubsystem, shooterSubsystem,
            RotationsPerSecond.of(20), Degrees.of(25), Degrees.of(180))));

    controlBindings.demoToss2().ifPresent(trigger -> trigger.whileTrue(
        new ManualShootCommand(turretSubsystem, shooterSubsystem,
            RotationsPerSecond.of(30), Degrees.of(20), Degrees.of(180))));

    controlBindings.seedFieldRelative().ifPresent(trigger -> trigger.onTrue(
        runOnce(drivetrain::seedFieldRelative, drivetrain)));
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
    // Intake
    columnIndex = 0;
    tab.add("Intake Quasi Fwd", intakeSubsystem.sysIdRollerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Intake Quasi Rev", intakeSubsystem.sysIdRollerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Intake Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Intake Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Shooter
    columnIndex += 2;
    tab.add("Shoot Quasi Fwd", shooterSubsystem.sysIdShooterQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Shoot Quasi Rev", shooterSubsystem.sysIdShooterQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Shoot Dynam Fwd", shooterSubsystem.sysIdShooterDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Shoot Dynam Rev", shooterSubsystem.sysIdShooterDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Turret yaw
    columnIndex += 2;
    tab.add("Tur Yaw Quasi Fwd", turretSubsystem.sysIdYawQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Tur Yaw Quasi Rev", turretSubsystem.sysIdYawQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Tur Yaw Dynam Fwd", turretSubsystem.sysIdYawDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Tur Yaw Dynam Rev", turretSubsystem.sysIdYawDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Turret pitch
    columnIndex += 2;
    tab.add("Tur Pitch Quasi Fwd", turretSubsystem.sysIdPitchQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Tur Pitch Quasi Rev", turretSubsystem.sysIdPitchQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Tur Pitch Dynam Fwd", turretSubsystem.sysIdPitchDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Tur Pitch Dynam Rev", turretSubsystem.sysIdPitchDynamicCommand(kReverse)).withPosition(columnIndex, 3);

    // Turret rollers
    columnIndex += 2;
    tab.add("Tur Roll Quasi Fwd", turretSubsystem.sysIdRollerQuasistaticCommand(kForward)).withPosition(columnIndex, 0);
    tab.add("Tur Roll Quasi Rev", turretSubsystem.sysIdRollerQuasistaticCommand(kReverse)).withPosition(columnIndex, 1);
    tab.add("Tur Roll Dynam Fwd", turretSubsystem.sysIdRollerDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Tur Roll Dynam Rev", turretSubsystem.sysIdRollerDynamicCommand(kReverse)).withPosition(columnIndex, 3);
  }

  public void populateTestingDashboard() {
    var tab = Shuffleboard.getTab("Testing");
    tab.addNumber("Number of Tests Run", () -> testCommand.getTestState());
    tab.addBoolean("Has Stopped", () -> testCommand.getHasStopped());
    tab.add("Start Testing", testCommand
        .deadlineWith(new LEDProgressBarCommand(ledSubsystem, testCommand::getTestState)));
  }

  public void setAlliance(Alliance alliance) {
    drivetrain.setOperatorPerspectiveForward(
        alliance == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
