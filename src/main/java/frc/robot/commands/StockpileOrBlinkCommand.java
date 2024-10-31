package frc.robot.commands;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static edu.wpi.first.wpilibj.util.Color.kRed;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_BLUE;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_INTERPOLATOR;
import static frc.robot.Constants.ShootingConstants.STOCKPILE_RED;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.led.LEDBlinkCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Command to make a choice between stockpiling or strobing the LEDs if the robot is not in the
 * mid-court between wings
 */
public class StockpileOrBlinkCommand extends Command {

  private final Command stockpile;
  private final Command dontShootCommand;
  private final CommandSwerveDrivetrain drivetrain;
  private Command selectedCommand;

  public StockpileOrBlinkCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      TurretSubsystem turretSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<LinearVelocity> xSupplier,
      Supplier<LinearVelocity> ySupplier,
      Supplier<AngularVelocity> rotationSupplier) {

    this.drivetrain = drivetrain;
    this.dontShootCommand =
        new FieldOrientedDriveCommand(drivetrain, xSupplier, ySupplier, rotationSupplier)
            .alongWith(new LEDBlinkCommand(ledSubsystem, kRed, 0.05));
    this.stockpile =
        new ShootTeleopCommand(
            drivetrain,
            shooter,
            turretSubsystem,
            ledSubsystem,
            xSupplier,
            ySupplier,
            STOCKPILE_RED,
            STOCKPILE_BLUE,
            STOCKPILE_INTERPOLATOR,
            0.3);

    addRequirements(drivetrain, shooter, turretSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance().map(dsAlliance -> dsAlliance).orElse(Alliance.Blue);
    Pose2d robotPose = drivetrain.getState().Pose;
    if (alliance == Red && robotPose.getX() > 5.3
        || alliance == Alliance.Blue && robotPose.getX() < 11.25) {
      selectedCommand = stockpile;
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
