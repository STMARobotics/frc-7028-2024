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

  public DriverTelemetry() {
    elevatorAtTopLimitEntry = driverTab.add("Elevator Top", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).getEntry();
    elevatorAtBottomLimitEntry = driverTab.add("Elevator Bottom", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).getEntry();
  }

  public void telemeterizeElevator(ElevatorState state) {
    elevatorAtTopLimitEntry.setBoolean(state.isAtTopLimit);
    elevatorAtBottomLimitEntry.setBoolean(state.isAtBottomLimit);
  }

}
