package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class CTREUtil {

  private static final double SLOW_UPDATE = 4.0;
  private static final double MEDIUM_UPDATE = 20.0;
  private static final double NORMAL_UPDATE = 100.0;

  /**
   * Optimizes signals for what we usually do with motors. Some signal frequencies are reduced so they use less
   * bandwidth but are still logged with SignalLogger, and others are turned off entirely.
   * 
   * @param motors motors to optimize
   */
  public static void optimizeSignals(TalonFX... motors) {
    // for (TalonFX motor : motors) {
    //   // Signals we use or we want to log frequently
    //   setUpdateFrequency(NORMAL_UPDATE,
    //       motor.getBridgeOutput(),
    //       motor.getClosedLoopDerivativeOutput(),
    //       motor.getClosedLoopError(),
    //       motor.getClosedLoopFeedForward(),
    //       motor.getClosedLoopOutput(),
    //       motor.getClosedLoopReference(),
    //       motor.getClosedLoopReferenceSlope(),
    //       motor.getDeviceTemp(),
    //       motor.getDutyCycle(),
    //       motor.getFaultField(),
    //       motor.getMotionMagicIsRunning(),
    //       motor.getMotorVoltage(),
    //       motor.getPosition(),
    //       motor.getRotorPosition(),
    //       motor.getRotorVelocity(),
    //       motor.getStatorCurrent(),
    //       motor.getSupplyCurrent(),
    //       motor.getSupplyVoltage(),
    //       motor.getStickyFaultField(),
    //       motor.getTorqueCurrent(),
    //       motor.getVelocity());

    //   // Signals we want to log, but not as often
    //   setUpdateFrequency(MEDIUM_UPDATE,
    //       motor.getAncillaryDeviceTemp(),
    //       motor.getAcceleration(),
    //       motor.getProcessorTemp());

    //   // Signals we only want to log infrequently
    //   setUpdateFrequency(SLOW_UPDATE,
    //       motor.getAppliedRotorPolarity(),
    //       motor.getClosedLoopIntegratedOutput(),
    //       motor.getClosedLoopSlot(),
    //       motor.getFault_BootDuringEnable(),
    //       motor.getForwardLimit(),
    //       motor.getIsProLicensed(),
    //       motor.getMotorType(),
    //       motor.getReverseLimit(),
    //       motor.getVersion(),
    //       motor.getVersionBugfix(),
    //       motor.getVersionBuild(),
    //       motor.getVersionMajor(),
    //       motor.getVersionMinor());      
    // }
    // // Turn off signals that aren't set above
    // ParentDevice.optimizeBusUtilizationForAll(motors);
  }

  @SuppressWarnings("rawtypes")
  private static void setUpdateFrequency(double frequency, StatusSignal... signals) {
    for (StatusSignal signal : signals) {
      signal.setUpdateFrequency(frequency);
    }
  }
}
