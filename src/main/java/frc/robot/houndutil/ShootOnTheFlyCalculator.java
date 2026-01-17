package com.techhounds.houndutil.houndlib;

import java.util.function.Function;

import com.techhounds.houndutil.houndlib.BallPhysics.ShotSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Provides static methods to calculate the effective target position to aim for
 * when shooting on the fly.
 */
public class ShootOnTheFlyCalculator {
    /**
     * Calculates the time it will take for a projectile to reach a target given the
     * robot's pose and the target's pose, and a function describing the
     * projectile's velocity. This allows you to have a shooter that may shoot a
     * projectile at varying speeds given varying distances from the targe.
     * 
     * @see #calculateEffectiveTargetLocation(Pose2d, Pose3d, ChassisSpeeds,
     *      ChassisAccelerations, Function, double, double)
     * 
     * @param robotPose                      the current pose of the robot
     * @param targetPose                     the 3D pose of the target. this should
     *                                       be the center of the target, if that
     *                                       makes sense (2024 game), but could also
     *                                       be offset if desired. this may also be
     *                                       deeper into the target area if required
     *                                       (2020 game). this should take into
     *                                       account any necessary field reflections
     *                                       before being passed.
     * @param xyDistanceToProjectileVelocity a function that takes in the
     *                                       xy-distance from the robot to the goal,
     *                                       and returns the velocity of the shot
     *                                       projectile in m/s.
     * @return the time it will take for the projectile to reach the target
     */
    public static double getTimeToShoot(Pose2d robotPose, Pose3d targetPose,
            Function<Double, Double> xyDistanceToProjectileVelocity) {
        Transform3d diff = new Pose3d(robotPose).minus(targetPose);
        double xyDistance = new Translation2d(diff.getX(), diff.getY()).getNorm();
        double distance = diff.getTranslation().getNorm();
        double projectileVelocity = xyDistanceToProjectileVelocity.apply(xyDistance);
        double time = distance / projectileVelocity;
        return time;
    }

    public static double getTimeToShoot(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed,
            double launchPitchRad) {
        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();

        double horizontalDist = Math.hypot(dx, dy);
        double vHoriz = launchSpeed * Math.cos(launchPitchRad);

        if (vHoriz <= 1e-6) {
            throw new IllegalArgumentException("Horizontal velocity too small");
        }

        return horizontalDist / vHoriz;
    }

    /**
     * Calculates the effective position of the target given the position, velocity,
     * and acceleration of the robot's chassis. Does not account for air resistance,
     * though this is very often unnecessary.
     * 
     * <p>
     * 
     * When shooting a projectile while moving, the projectile inherits the
     * translational velocity of the chassis. Shooting on the fly can be
     * accomplished by targeting a "virtual" goal if you are moving, which acts to
     * negate the forces applied on the projectile due to the movement of the
     * chassis.
     * 
     * <p>
     * 
     * An iterative approach (see {@code goalPositionIterations}) is required
     * because the time taken for the projectile to travel to the target will change
     * given a different target location. To account for this, we re-simulate the
     * projectile's travel with a new shot time derived from the new virtual goal
     * position several times.
     * 
     * 
     * <p>
     * 
     * When solving this problem mathematically, the acceleration of the chassis
     * does not matter in the final velocities of the projectile (it will not
     * inherit the acceleration of the chassis). The
     * {@code accelerationCompensationFactor} is necessary due to other errors:
     * 
     * (1) the time taken to move the projectile through a shooter is non-zero, so
     * (2) if the chassis is accelerating, the velocity of the chassis by the time
     * the projectile leaves the robot will have changed.
     * 
     * To account for this, we multiply the acceleration at the time of the shot
     * <i>command</i> by a specific value and add it to the velocity at the time of
     * the shot. This value is based on the time delta between
     * commanding a shot and the shot actually leaving the shooter, meaning that the
     * effective velocity generated is the velocity as the projectile leaves the
     * shooter. This is extremely complicated to determine theoretically, so if you
     * find acceleration to be causing shot inaccuracies, find a value that provides
     * adequate compensation (should be around [0,2]).
     * 
     * <p>
     * 
     * The easiest way to create the {@code xyDistanceToProjectileVelocity} lambda
     * function is as follows:
     * 
     * If the speed of your shooter is always constant, simply create a lambda
     * expression that always returns the same value.
     * 
     * If the speed of your shooter is controlled using an
     * {@link InterpolatingTreeMap} based on distance from the goal, simply get the
     * appropriate shooter speed from that map, and multiply it by some constant
     * that describes how fast the projectile moves given a shooter speed. This can
     * be calculated experimentally by pointing a camera at the shooter and
     * calculating the speed of the projectile based on the distance travelled in n
     * frames. If you find the relationship between shooter speed and projectile
     * speed is not constant, you can create a second {@link InterpolatingTreeMap},
     * or define it as an equation.
     * 
     * @param robotPose                      the current pose of the robot
     * @param targetPose                     the 3D pose of the target. this should
     *                                       be the center of the target, if that
     *                                       makes sense (2024 game), but could also
     *                                       be offset if desired. this may also be
     *                                       deeper into the target area if required
     *                                       (2020 game). this should take into
     *                                       account any necessary field reflections
     *                                       before being passed.
     * @param fieldRelRobotVelocity          the field-relative velocity of the
     *                                       robot's chassis
     * @param fieldRelRobotAcceleration      the field-relative acceleration of the
     *                                       robot's chassis
     * @param xyDistanceToProjectileVelocity a function that takes in the
     *                                       xy-distance from the robot to the goal,
     *                                       and returns the velocity of the shot
     *                                       projectile in m/s.
     * @param goalPositionIterations         the number of iterations to use when
     *                                       iteratively solving for the pose of the
     *                                       target. a higher number of iterations
     *                                       will increase the accuracy of the
     *                                       result, but will also reduce
     *                                       performance.
     * @param accelerationCompensationFactor the value to multiply the acceleration
     * @return
     */
    public static Pose3d calculateEffectiveTargetLocation(
            Pose2d robotPose, Pose3d targetPose,
            ChassisSpeeds fieldRelRobotVelocity,
            ChassisAccelerations fieldRelRobotAcceleration,
            Function<Double, Double> xyDistanceToProjectileVelocity,
            double goalPositionIterations,
            double accelerationCompensationFactor) {

        double shotTime = getTimeToShoot(robotPose, targetPose, xyDistanceToProjectileVelocity);

        Pose3d correctedTargetPose = new Pose3d();
        for (int i = 0; i < goalPositionIterations; i++) {
            double virtualGoalX = targetPose.getX()
                    - shotTime * (fieldRelRobotVelocity.vxMetersPerSecond
                            + fieldRelRobotAcceleration.axMetersPerSecondSquared
                                    * accelerationCompensationFactor);
            double virtualGoalY = targetPose.getY()
                    - shotTime * (fieldRelRobotVelocity.vyMetersPerSecond
                            + fieldRelRobotAcceleration.ayMetersPerSecondSquared
                                    * accelerationCompensationFactor);

            correctedTargetPose = new Pose3d(virtualGoalX, virtualGoalY, targetPose.getZ(),
                    targetPose.getRotation());

            double newShotTime = getTimeToShoot(robotPose, correctedTargetPose, xyDistanceToProjectileVelocity);

            shotTime = newShotTime;
            if (Math.abs(newShotTime - shotTime) <= 0.010) {
                break;
            }
        }

        return correctedTargetPose;
    }

    public record InterceptSolution(
            Pose3d effectiveTargetPose,
            double launchPitchRad,
            double launchSpeed,
            double flightTime,
            double requiredYaw) {
    }

    public static InterceptSolution solveShootOnTheFly(
            Pose3d shooterPose,
            Pose3d targetPose,
            ChassisSpeeds fieldRelRobotVelocity,
            ChassisAccelerations fieldRelRobotAcceleration,
            double targetSpeedRps,
            int maxIterations,
            double timeTolerance) {

        ShotSolution sol = BallPhysics.solveBallisticWithSpeed(
                shooterPose,
                targetPose,
                targetSpeedRps);

        double t = sol.flightTimeSeconds();
        Pose3d effectiveTarget = targetPose;

        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.axMetersPerSecondSquared * t * t;

            double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.ayMetersPerSecondSquared * t * t;

            effectiveTarget = new Pose3d(
                    targetPose.getX() - dx,
                    targetPose.getY() - dy,
                    targetPose.getZ(),
                    targetPose.getRotation());

            ShotSolution newSol = BallPhysics.solveBallisticWithSpeed(
                    shooterPose,
                    effectiveTarget,
                    targetSpeedRps);

            if (Math.abs(newSol.flightTimeSeconds() - t) < timeTolerance) {
                return new InterceptSolution(
                        effectiveTarget,
                        newSol.launchPitchRad(),
                        newSol.launchSpeed(),
                        newSol.flightTimeSeconds(),
                        0);
            }

            sol = newSol;
            t = newSol.flightTimeSeconds();
        }

        return new InterceptSolution(
                effectiveTarget,
                sol.launchPitchRad(),
                sol.launchSpeed(),
                sol.flightTimeSeconds(),
                0);
    }
}