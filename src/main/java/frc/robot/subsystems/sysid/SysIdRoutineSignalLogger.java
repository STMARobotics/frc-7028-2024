package frc.robot.subsystems.sysid;

import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class SysIdRoutineSignalLogger extends SignalLogger {
    public static Consumer<SysIdRoutineLog.State> logState() {
        start(); // Start logging if we get the consumer, so we have some data before the start of the motion
        return (SysIdRoutineLog.State state) -> writeString("State", state.toString());
    }

    public static void registerAsSysIdLog(ParentDevice device) {
        writeInteger("SysId Logged Device", device.getDeviceHash(), "");
    }
}
