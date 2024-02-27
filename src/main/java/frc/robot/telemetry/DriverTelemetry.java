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
  
  private final GenericEntry isShooterReadyEntry;
  private final GenericEntry isDrivetrainStoppedEntry;
  private final GenericEntry isYawReadyEntry;
  private final GenericEntry isPitchReadyEntry;
  private final GenericEntry isInTurretRangeEntry;
  private final GenericEntry targetDistanceEntry;
  private final GenericEntry targetAngleDegreesEntry;
  private final GenericEntry turretAngleDegreesEntry;

  public DriverTelemetry() {

    var shootingList = driverTab.getLayout("Shooting", BuiltInLayouts.kList).withSize(2, 6).withPosition(8, 0);
    isShooterReadyEntry = shootingList.add("shooter", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).getEntry();
    isDrivetrainStoppedEntry = shootingList.add("stopped", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 1).getEntry();
    isYawReadyEntry = shootingList.add("yaw", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2).getEntry();
    isPitchReadyEntry = shootingList.add("pitch", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 3).getEntry();
    isInTurretRangeEntry = shootingList.add("turret range", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 4).getEntry();
    targetDistanceEntry = shootingList.add("distance", 0.0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 5).getEntry();
    targetAngleDegreesEntry = shootingList.add("angle", 0.0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 6).getEntry();
    turretAngleDegreesEntry = shootingList.add("turret angle", 0.0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 6).getEntry();
  }

  public void telemeterizeShooting(ShootingState state) {
    isShooterReadyEntry.setBoolean(state.isShooterReady);;
    isDrivetrainStoppedEntry.setBoolean(state.isDrivetrainStopped);
    isYawReadyEntry.setBoolean(state.isYawReady);
    isPitchReadyEntry.setBoolean(state.isPitchReady);
    isInTurretRangeEntry.setBoolean(state.isInTurretRange);
    targetDistanceEntry.setDouble(state.targetDistance);
    targetAngleDegreesEntry.setDouble(state.targetAngleDegrees);
    turretAngleDegreesEntry.setDouble(state.turretAngleDegrees);
  }

}
