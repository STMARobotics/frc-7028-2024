package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * A 2D rectangle defined by two points. Used primarily for collision detection
 * or pathfinding.
 */
public class Rectangle2d {
    private Translation2d topLeft;
    private Translation2d bottomRight;

    /**
     * Create a new Rectangle2d.
     * 
     * <p>
     * The "top" of the rectangle is defined as +x, and the "left" side is defined
     * as +y.
     * 
     * @param topLeft
     * @param topRight
     * @param bottomLeft
     * @param bottomRight
     */
    public Rectangle2d(Pose2d topLeft, Pose2d bottomRight) {
        this.topLeft = topLeft.getTranslation();
        this.bottomRight = bottomRight.getTranslation();
    }

    /**
     * Checks if a specific pose is within the rectangle.
     * 
     * @param point the pose to check
     * @return whether the pose is in the rectangle
     */
    public boolean isInRect(Pose2d point) {
        return (point.getX() < topLeft.getX() && point.getX() > bottomRight.getX())
                && (point.getY() < topLeft.getY() && point.getY() > bottomRight.getY());
    }

    /**
     * Draws the rectangle on a Field2d object.
     * 
     * @param field the Field2d object to draw on
     */
    public void drawOnField(Field2d field) {
        field.getObject(topLeft.toString() + " " + bottomRight.toString()).setPoses(
                new Pose2d(topLeft, Rotation2d.kZero),
                new Pose2d(topLeft.getX(), bottomRight.getY(), Rotation2d.kZero),
                new Pose2d(bottomRight, Rotation2d.kZero),
                new Pose2d(bottomRight.getX(), topLeft.getY(), Rotation2d.kZero),
                new Pose2d(topLeft, Rotation2d.kZero));
    }
}
