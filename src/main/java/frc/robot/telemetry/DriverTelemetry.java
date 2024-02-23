package frc.robot.telemetry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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
  private final GenericEntry elevatorMetersEntry;

  private final GenericEntry isShooterReadyEntry;
  private final GenericEntry isDrivetrainStoppedEntry;
  private final GenericEntry isYawReadyEntry;
  private final GenericEntry isPitchReadyEntry;
  private final GenericEntry isInTurretRangeEntry;
  private final GenericEntry targetDistanceEntry;
  private final GenericEntry targetAngleDegreesEntry;

  public DriverTelemetry() {
    elevatorAtTopLimitEntry = driverTab.add("Elevator Top", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 0).getEntry();
    elevatorAtBottomLimitEntry = driverTab.add("Elevator Bottom", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).getEntry();
    elevatorMetersEntry = driverTab.add("Elevator Meters", 0.0)
        .withWidget(BuiltInWidgets.kTextView).withPosition(4, 0).getEntry();
    

    var shootingList = driverTab.getLayout("Shooting", BuiltInLayouts.kList).withPosition(5, 0);
    isShooterReadyEntry = shootingList.add("shooter", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).getEntry();
    isDrivetrainStoppedEntry = shootingList.add("stopped", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1).getEntry();
    isYawReadyEntry = shootingList.add("yaw", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2).getEntry();
    isPitchReadyEntry = shootingList.add("pitch", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 3).getEntry();
    isInTurretRangeEntry = shootingList.add("turret range", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 4).getEntry();
    targetDistanceEntry = shootingList.add("distance", 0.0).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 5).getEntry();
    targetAngleDegreesEntry = shootingList.add("angle", 0.0).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 6).getEntry();
  }

  public void telemeterizeElevator(ElevatorState state) {
    elevatorAtTopLimitEntry.setBoolean(state.isAtTopLimit);
    elevatorAtBottomLimitEntry.setBoolean(state.isAtBottomLimit);
    elevatorMetersEntry.setDouble(state.elevatorMeters);
  }

  public void telemeterizeShooting(ShootingState state) {
    isShooterReadyEntry.setBoolean(state.isShooterReady);;
    isDrivetrainStoppedEntry.setBoolean(state.isDrivetrainStopped);
    isYawReadyEntry.setBoolean(state.isYawReady);
    isPitchReadyEntry.setBoolean(state.isPitchReady);
    isInTurretRangeEntry.setBoolean(state.isInTurretRange);
    targetDistanceEntry.setDouble(state.targetDistance);
    targetAngleDegreesEntry.setDouble(state.targetAngleDegrees);
  }

}
