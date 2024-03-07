package frc.robot.commands.testing;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {
  
  private final IntSupplier intSupplier;
  private final LEDSubsystem ledSubsystem;
  private int index;
  private int tests;

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem, IntSupplier intSupplier) {
    this.ledSubsystem = ledSubsystem;
    this.intSupplier = intSupplier;
    
    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {

    ledSubsystem.setUpdater(this::animate);
  }

  private void animate(LEDStrips ledStrips) { 
    for(int strip = 0; strip < ledStrips.getStripCount(); strip++) {
      for(index = 0; index < ledStrips.getStripSize(); index++) { 
        if (index<tests*2) {
          ledStrips.setLED(strip, index, Color.kBlue);
        } else {
          ledStrips.setLED(strip, index, Color.kBlack);
        }
      }
    }
    ledStrips.refresh();
    }

  @Override
  public void execute() {
    tests = intSupplier.getAsInt();
  }
    
  @Override
  public boolean isFinished() {
    return !RobotState.isTest();
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setUpdater(null);
  }
}
