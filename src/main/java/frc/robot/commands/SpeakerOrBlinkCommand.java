package frc.robot.commands;

import static edu.wpi.first.wpilibj.util.Color.kPurple;
import static frc.robot.Constants.ShootingConstants.SHOOTER_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BLUE_TELE;
import static frc.robot.Constants.ShootingConstants.SPEAKER_RED_TELE;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.led.LEDBlinkCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Command to make a choice between shooting or strobing the LEDs if there's no note in the turret
 */
public class SpeakerOrBlinkCommand extends Command {

  private final Command shootCommand;
  private final Command dontShootCommand;
  private final TurretSubsystem turretSubsystem;
  private Command selectedCommand;

  public SpeakerOrBlinkCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<LinearVelocity> xSupplier,
      Supplier<LinearVelocity> ySupplier,
      Supplier<AngularVelocity> rotationSupplier) {

    this.turretSubsystem = turretSubsystem;
    this.dontShootCommand = new FieldOrientedDriveCommand(drivetrain, xSupplier, ySupplier, rotationSupplier)
        .alongWith(new LEDBlinkCommand(ledSubsystem, kPurple, 0.05));
    this.shootCommand = new ShootTeleopCommand(
        drivetrain,
        shooter,
        turretSubsystem,
        ledSubsystem,
        xSupplier,
        ySupplier,
        SPEAKER_RED_TELE,
        SPEAKER_BLUE_TELE,
        SHOOTER_INTERPOLATOR,
        0);

    addRequirements(drivetrain, shooter, turretSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    if (turretSubsystem.hasNote()) {
      selectedCommand = shootCommand;
    } else {
      selectedCommand = dontShootCommand;
    }
    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    selectedCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    selectedCommand.end(interrupted);
  }
}
