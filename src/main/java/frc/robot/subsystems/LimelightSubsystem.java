package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;

  public LimelightSubsystem(String networkTableName) {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
  }
  
  public String getNetworkTableName() {
    return networkTableName;
  }

}