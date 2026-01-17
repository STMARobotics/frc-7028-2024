package com.techhounds.houndutil.houndlib;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.List;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

@LoggedObject
public class BallSimulator {
    private static final double OUT_OF_BOUNDS_MARGIN = 1.0;
    private static final double FIXED_DT = 0.001; // 1 ms
    private static final double MAX_FRAME_TIME = 0.05; // 50 ms safety clamp

    private long lastUpdateMicros = System.nanoTime();
    private double timeAccumulator = 0.0;

    private final BallConstants constants;
    private final List<BallState> balls = new ArrayList<>();

    private final double fieldLength;
    private final double fieldWidth;

    public BallSimulator(
            BallConstants constants,
            double fieldLength,
            double fieldWidth) {

        this.constants = constants;
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
    }

    public int addBall(BallState initialState) {
        balls.add(initialState);
        return balls.size() - 1;
    }

    public void update() {
        long now = RobotController.getFPGATime();
        double frameDt = (now - lastUpdateMicros) * 1e-6;
        lastUpdateMicros = now;

        if (frameDt <= 0.0) {
            return;
        }

        frameDt = Math.min(frameDt, MAX_FRAME_TIME);

        timeAccumulator += frameDt;

        while (timeAccumulator >= FIXED_DT) {
            for (BallState ball : balls) {
                BallPhysics.step(ball, constants, FIXED_DT);
            }
            timeAccumulator -= FIXED_DT;
        }

        if (timeAccumulator > 0.0) {
            for (BallState ball : balls) {
                BallPhysics.step(ball, constants, timeAccumulator);
            }
            timeAccumulator = 0.0;
        }

        for (int i = balls.size() - 1; i >= 0; i--) {
            if (isOutOfBounds(balls.get(i).pose.getTranslation())) {
                balls.remove(i);
            }
        }
    }

    @Log
    public Pose3d[] getBallPoses() {
        Pose3d[] poses = new Pose3d[balls.size()];

        for (int i = 0; i < balls.size(); i++) {
            poses[i] = balls.get(i).pose;
        }

        return poses;
    }

    private boolean isOutOfBounds(Translation3d p) {
        return p.getZ() < -OUT_OF_BOUNDS_MARGIN;
        // p.getX() < -OUT_OF_BOUNDS_MARGIN ||
        // p.getY() < -OUT_OF_BOUNDS_MARGIN ||
        // p.getX() > fieldLength + OUT_OF_BOUNDS_MARGIN ||
        // p.getY() > fieldWidth + OUT_OF_BOUNDS_MARGIN;
    }
}