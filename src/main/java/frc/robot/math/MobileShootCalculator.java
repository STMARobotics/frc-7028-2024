package frc.robot.math;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MobileShootCalculator {
  
  private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  ArrayList<Double> currentSpeeds = new ArrayList<Double>();

  public MobileShootCalculator() {
    currentSpeeds.add(getXSpeed());
    currentSpeeds.add(getYSpeed());
  }

  public double getXSpeed() {
    var fieldSpeeds = 
        new Translation2d(chassisSpeeds.vxMetersPerSecond, 0);
    return fieldSpeeds.getX();
  }

  public double getYSpeed() {
    var fieldSpeeds = 
        new Translation2d(0, chassisSpeeds.vyMetersPerSecond);
    return fieldSpeeds.getY();
  }

  public double getYaw() {
    return 90-Math.atan((currentSpeeds.get(0)/currentSpeeds.get(1)));
  }

}
