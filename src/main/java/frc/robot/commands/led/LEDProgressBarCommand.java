package frc.robot.commands.led;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDProgressBarCommand extends Command {
  
  private final LEDSubsystem ledSubsystem;
  private boolean done;
  private int index;
  private double tests;
  
  ShuffleboardTab tab = Shuffleboard.getTab("Testing");
       
  GenericEntry testsCompleted =
  tab.add("Tests Completed", 0).withPosition(0, 2).getEntry();

  public LEDProgressBarCommand(LEDSubsystem ledSubsystem) {

    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {

    ledSubsystem.setUpdater(this::animate);
  }

  public void UpdateTestingDashboard() {
    
    tests = testsCompleted.getDouble(0);
  }

  private void animate(LEDStrips ledStrips) {
      for(int strip = 0; strip < ledStrips.getStripCount(); strip++) {
        for(index = 0; index < tests*2; index++) {
            ledStrips.setLED(strip, index, Color.kBlue);
        }
      }
      ledStrips.refresh();
      done = !RobotState.isTest();
    }

  @Override
  public void execute() {
    UpdateTestingDashboard();
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
