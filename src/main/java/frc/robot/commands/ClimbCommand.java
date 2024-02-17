
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClimbCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final DoubleConsumer rumble;
  private final BooleanSupplier turretIsClear;

  public ClimbCommand(
      ElevatorSubsystem elevatorSubsystem,
      DoubleSupplier firstStageSupplier,
      DoubleConsumer rumble,
      BooleanSupplier turretIsClear) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.rumble = rumble == null ? (r) -> {} : rumble;
    this.turretIsClear = turretIsClear;
    
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    var requestedSpeed = MathUtil.applyDeadband(-0.25,0.5);
    if (requestedSpeed == 0) {
      elevatorSubsystem.stopClimb();
      rumble.accept(0d);
    } else {
      if (turretIsClear.getAsBoolean()) {
        // turret is clear so move (the subsystem also enforces this)
        elevatorSubsystem.climbUp();
        rumble.accept(0d);
      } else {
        // turret is not clear, so do not move and rumble the controller
        elevatorSubsystem.stopClimb();
        rumble.accept(1d);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopClimb();
    rumble.accept(0d);
  }
  
}