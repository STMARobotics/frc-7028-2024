package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class BallPhysics {
    public static final double GRAVITY = 9.81;

    public record ShotSolution(
            double launchPitchRad,
            double launchSpeed,
            double flightTimeSeconds) {
    }

    private BallPhysics() {
    }

    private static Translation3d gravityForce(BallConstants c) {
        return new Translation3d(0, 0, -c.mass * c.gravity);
    }

    private static Translation3d dragForce(
            Translation3d v, BallConstants c) {

        double speed = v.getNorm();
        if (speed < 1e-6)
            return new Translation3d();

        double scale = -0.5 * c.rho * c.cd * c.area * speed;
        return v.times(scale);
    }

    private static Translation3d magnusForce(
            Translation3d v, Translation3d omega, BallConstants c) {

        double speed = v.getNorm();
        double wMag = omega.getNorm();
        if (speed < 1e-6 || wMag < 1e-6)
            return new Translation3d();

        double spinRatio = wMag * c.radius / speed;
        double cl = Math.min(c.clGain * spinRatio, c.clMax);

        Translation3d vHat = v.div(speed);
        Translation3d wHat = omega.div(wMag);

        Translation3d direction = new Translation3d(Vector.cross(wHat.toVector(), vHat.toVector()));
        double magnitude = 0.5 * c.rho * cl * c.area * speed * speed;

        return direction.times(magnitude);
    }

    private static Rotation3d integrateRotation(
            Rotation3d current,
            Translation3d omega,
            double dt) {

        Rotation3d delta = new Rotation3d(
                omega.getX() * dt,
                omega.getY() * dt,
                omega.getZ() * dt);

        return current.plus(delta);
    }

    public static void step(
            BallState s, BallConstants c, double dt) {

        Translation3d force = gravityForce(c)
                .plus(dragForce(s.velocity, c))
                .plus(magnusForce(s.velocity, s.omega, c));

        Translation3d accel = force.div(c.mass);

        double decay = Math.exp(-dt / c.spinDecayTau);
        s.omega = s.omega.times(decay);

        s.velocity = s.velocity.plus(accel.times(dt));

        s.pose = new Pose3d(s.pose.getTranslation().plus(s.velocity.times(dt)),
                integrateRotation(s.pose.getRotation(), s.omega, dt));
    }

    public static ShotSolution solveBallisticWithIncomingAngle(
            Pose3d shooterPose,
            Pose3d targetPose,
            double incomingPitchRad) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double tanThetaT = Math.tan(incomingPitchRad);

        double rhs = dz - d * tanThetaT;
        if (rhs <= 0) {
            throw new IllegalArgumentException(
                    "No physical solution: dz - d*tan(thetaT) must be > 0");
        }

        double T = Math.sqrt(2.0 * rhs / GRAVITY);

        double vHoriz = d / T;
        double vZ0 = vHoriz * tanThetaT + GRAVITY * T;

        double launchSpeed = Math.hypot(vHoriz, vZ0);
        double launchPitch = Math.atan2(vZ0, vHoriz);

        return new ShotSolution(launchPitch, launchSpeed, T);
    }

    public static ShotSolution solveBallisticWithSpeed(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double v2 = launchSpeed * launchSpeed;
        double g = GRAVITY;

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, 0);
        }

        // LOW-ARC solution (use +Math.sqrt(...) for high arc)
        double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeed * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeed, time);
    }

    public static double minSpeedForAnyArc(
            Pose3d shooterPose,
            Pose3d targetPose) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);

        return Math.sqrt(
                GRAVITY * (Math.hypot(d, dz) + dz));
    }
}