package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

/**
 * Struct for publishing AprilTag data to the NetworkTables. Advantage Scope can render this.
 * See https://www.chiefdelphi.com/t/publishing-apriltags-for-advantagescope/455749
 */
public class AprilTagStruct implements Struct<AprilTag> {
  @Override
  public Class<AprilTag> getTypeClass() {
    return AprilTag.class;
  }

  @Override
  public String getTypeString() {
    return "struct:AprilTag";
  }

  @Override
  public int getSize() {
    return kSizeInt8 + Pose3d.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "uint8 ID;Pose3d pose";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose3d.struct};
  }

  @Override
  public AprilTag unpack(ByteBuffer bb) {
    int id = (int) bb.get();
    Pose3d pose = Pose3d.struct.unpack(bb);
    return new AprilTag(id, pose);
  }

  @Override
  public void pack(ByteBuffer bb, AprilTag value) {
    bb.put((byte) value.ID);
    Pose3d.struct.pack(bb, value.pose);
  }
}