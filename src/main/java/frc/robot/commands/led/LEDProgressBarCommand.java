package frc.robot.commands.led;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {
  
  private final RobotContainer robotContainer;
  private final LEDSubsystem ledSubsystem;
  private boolean done;
  private int index;

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, RobotContainer robotContainer) {
    this.ledSubsystem = ledSubsystem;
    this.robotContainer = robotContainer;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {

    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
      for(int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        for(index = 0; index < robotContainer.getTestsDone(); index++) {
            ledStrips.setLED(strip, index, Color.kBlue);
        }
      }
      ledStrips.refresh();
      done = !RobotState.isTest();
    }

    @Override
    public void execute() {

    }
    
  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
