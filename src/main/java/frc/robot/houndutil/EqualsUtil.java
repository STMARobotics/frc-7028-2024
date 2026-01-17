package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Provides utility methods for comparing numerical values.
 * 
 * @see com.techhounds.houndutil.houndlib.swerve.SwerveSetpointGenerator
 */
public class EqualsUtil {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
        return EqualsUtil.epsilonEquals(twist.dx, other.dx)
                && EqualsUtil.epsilonEquals(twist.dy, other.dy);
    }
}
