package frc.robot.telemetry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Manages the Driver tab on the dashboard
 */
public class DriverTelemetry {

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  
  private final GenericEntry elevatorAtTopLimitEntry;
  private final GenericEntry elevatorAtBottomLimitEntry;
  private final GenericEntry indexerHasNoteEntry;

  public DriverTelemetry() {
    elevatorAtTopLimitEntry = driverTab.add("Elevator Top", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).getEntry();
    elevatorAtBottomLimitEntry = driverTab.add("Elevator Bottom", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).getEntry();
    indexerHasNoteEntry = driverTab.add("Has Note", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 1).getEntry();
  }

  public void telemeterizeElevator(ElevatorState state) {
    elevatorAtTopLimitEntry.setBoolean(state.isAtTopLimit);
    elevatorAtBottomLimitEntry.setBoolean(state.isAtBottomLimit);
  }

  public void telemeterizeIndexer(boolean hasNote) {
    indexerHasNoteEntry.setBoolean(hasNote);
  }

}
