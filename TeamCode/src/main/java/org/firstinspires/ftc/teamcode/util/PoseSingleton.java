package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseSingleton {
    private static final PoseSingleton instance = new PoseSingleton();
    Pose2d pose = new Pose2d(0, 0, 0);

    private PoseSingleton() {}

    public static PoseSingleton getInstance() {
        return instance;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
