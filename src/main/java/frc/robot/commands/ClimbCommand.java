
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

  private final ClimbSubsystem climbSubsystem;
  private final DoubleConsumer rumble;
  private final BooleanSupplier turretIsClear;

  public ClimbCommand(
      ClimbSubsystem climbSubsystem,
      DoubleConsumer rumble,
      BooleanSupplier turretIsClear) {
    this.climbSubsystem = climbSubsystem;
    this.rumble = rumble == null ? (r) -> {} : rumble;
    this.turretIsClear = turretIsClear;
    
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    var requestedSpeed = MathUtil.applyDeadband(-0.25,0.5);
    if (requestedSpeed == 0) {
      climbSubsystem.stopClimb();
      rumble.accept(0d);
    } else {
      if (turretIsClear.getAsBoolean()) {
        // turret is clear so move (the subsystem also enforces this)
        climbSubsystem.climbUp();
        rumble.accept(0d);
      } else {
        // turret is not clear, so do not move and rumble the controller
        climbSubsystem.stopClimb();
        rumble.accept(1d);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimb();
    rumble.accept(0d);
  }
  
}
