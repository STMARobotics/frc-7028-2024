package frc.robot.subsystems.sysid;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

/**
 * SwerveRequest used for SysId. Allows a given voltage to be applied to:
 * <p>- the drive motors while the steer motors are held at zero, or </p>
 * <p>- the steer motors while the drive motors are held at zero</p>
 */
public class VoltageSwerveRequest implements SwerveRequest {
    private final MotionMagicVoltage m_motionMagicControl = 
        new MotionMagicVoltage(0, false, 0, 0, false, false, false);
    private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);

    private double m_targetVoltage = 0.0;
    private boolean m_driveType = true;

    public VoltageSwerveRequest() {
        m_driveType = true;
    }

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (var module : modulesToApply) 
        {
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl);

                // Command drive motor to voltage
                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));
            }
            else {
                // Command steer motor to voltage
                module.getSteerMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));

                // Command drive motor to zero voltage
                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(0.0));
            }
        }

        return StatusCode.OK;
    }

    /**
     * Applies the requested voltage to the steer motors, and holds drive motors at 0
     * @param targetVoltage voltage for steer motors
     * @return this
     */
    public VoltageSwerveRequest withSteerVoltage(Measure<Voltage> targetVoltage) {
        this.m_targetVoltage = targetVoltage.in(Volts);
        this.m_driveType = false;
        return this;
    }

    /**
     * Applies the requested voltage to the drive motors, and holds steer motor at 0
     * @param targetVoltage voltage for drive motors
     * @return this
     */
    public VoltageSwerveRequest withDriveVoltage(Measure<Voltage> targetVoltage) {
        this.m_targetVoltage = targetVoltage.in(Volts);
        this.m_driveType = true;
        return this;
    }

}