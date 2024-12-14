package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseKalmanFilter {
    Pose2d poseEstimate;
    double modelCovariance, visionCovariance;
    double p, K;
    double lastUpdate = System.currentTimeMillis();

    public PoseKalmanFilter(Pose2d poseEstimate, double modelCovariance, double visionCovariance) {
        this.poseEstimate = poseEstimate;
        this.modelCovariance = modelCovariance;
        this.visionCovariance = visionCovariance;
        this.p = 1;
        this.K = 1;
    }
    
    public PoseKalmanFilter(double modelCovariance, double visionCovariance) {
        this(new Pose2d(0, 0, 0), modelCovariance, visionCovariance);
    }

    // run only when a vision update is available
    public Pose2d update(Pose2d poseUpdate, Pose2d visionUpdate) {
        p += modelCovariance * ((System.currentTimeMillis() - lastUpdate) / 1000);
        lastUpdate = System.currentTimeMillis();

        K = p / (p + visionCovariance);

        poseEstimate = new Pose2d(
                poseUpdate.position.x + (K * (visionUpdate.position.x - poseUpdate.position.x)),
                poseUpdate.position.y + (K * (visionUpdate.position.y - poseUpdate.position.y)),
                poseUpdate.heading.toDouble()
        );
        
        p = (1 - K) * p;
        
        return poseEstimate;
    }

    // run when no new vision pose is available
    public Pose2d predict(Pose2d poseUpdate) {
        p += modelCovariance * ((System.currentTimeMillis() - lastUpdate) / 1000);
        lastUpdate = System.currentTimeMillis();
        return poseUpdate;
    }
}
