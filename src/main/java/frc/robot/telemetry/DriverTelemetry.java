package frc.robot.telemetry;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
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
  private final GenericEntry intakePositionEntry;

  public DriverTelemetry() {
    elevatorAtTopLimitEntry = driverTab.add("Elevator Top", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).getEntry();
    elevatorAtBottomLimitEntry = driverTab.add("Elevator Bottom", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).getEntry();
    indexerHasNoteEntry = driverTab.add("Has Note", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 1).getEntry();
    intakePositionEntry = driverTab.add("Intake Position", 0.0)
        .withWidget(BuiltInWidgets.kTextView).withPosition(4, 0).getEntry();
  }

  public void telemeterizeElevator(ElevatorState state) {
    elevatorAtTopLimitEntry.setBoolean(state.isAtTopLimit);
    elevatorAtBottomLimitEntry.setBoolean(state.isAtBottomLimit);
  }

  public void telemeterizeIndexer(boolean hasNote) {
    indexerHasNoteEntry.setBoolean(hasNote);
  }

  public void telemeterizeIntake(Measure<Angle> position) {
    intakePositionEntry.setDouble(position.in(Rotations));
  }

}
