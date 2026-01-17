package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Represents the complete acceleration data of a chassis. Counterpart to
 * {@link ChassisSpeeds}. Used when acceleration data is needed for some
 * computation (in 2024, shooting on the move).
 */
public class ChassisAccelerations {
    public double axMetersPerSecondSquared;
    public double ayMetersPerSecondSquared;
    public double omegaRadiansPerSecondSquared;

    /**
     * Creates a {@link ChassisAccelerations} object based on given acceleration
     * data.
     * 
     * @param axMetersPerSecondSquared     the acceleration in the x direction, in
     *                                     m/s^2
     * @param ayMetersPerSecondSquared     the acceleration in the y direction, in
     *                                     m/s^2
     * @param omegaRadiansPerSecondSquared the rotational acceleration of the
     *                                     chassis (CCW+), in rad/s^2
     */
    public ChassisAccelerations(double axMetersPerSecondSquared, double ayMetersPerSecondSquared,
            double omegaRadiansPerSecondSquared) {
        this.axMetersPerSecondSquared = axMetersPerSecondSquared;
        this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
        this.omegaRadiansPerSecondSquared = omegaRadiansPerSecondSquared;
    }

    /**
     * Creates a {@link ChassisAccelerations} object based on the current
     * {@link ChassisSpeeds}, the {@link ChassisSpeeds} from the previous iteration,
     * and the time between iterations.
     * 
     * @param speed         the current speed of the chassis
     * @param previousSpeed the previous speed of the chassis
     * @param dt            the time elapsed between measurements (your loop time),
     *                      in seconds
     */
    public ChassisAccelerations(ChassisSpeeds speed, ChassisSpeeds previousSpeed, double dt) {
        this.axMetersPerSecondSquared = (speed.vxMetersPerSecond - previousSpeed.vxMetersPerSecond) / dt;
        this.ayMetersPerSecondSquared = (speed.vyMetersPerSecond - previousSpeed.vyMetersPerSecond) / dt;
        this.omegaRadiansPerSecondSquared = (speed.omegaRadiansPerSecond - previousSpeed.omegaRadiansPerSecond) / dt;
    }
}