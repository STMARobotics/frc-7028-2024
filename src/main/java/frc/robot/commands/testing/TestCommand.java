package frc.robot.commands.testing;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class TestCommand extends Command {
  
  private final LEDSubsystem ledSubsystem;

  private ShuffleboardTab tab = Shuffleboard.getTab("Testing");
       
  public GenericEntry testsCompleted = tab.add("Tests Completed", 0).withPosition(0, 1).getEntry();

  public TestCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    tab.add("Start LED Progress Bar", new LEDProgressBarCommand(ledSubsystem, this)).withPosition(0, 0);
  }
}
