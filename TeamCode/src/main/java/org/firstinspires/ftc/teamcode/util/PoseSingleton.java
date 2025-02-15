package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseSingleton {
    private static final PoseSingleton instance = new PoseSingleton();
    static Pose2d pose = new Pose2d(0, 0, 0);
    static double armAngle = -40;

    private PoseSingleton() {}

    public static PoseSingleton getInstance() {
        return instance;
    }

    public Pose2d getPose() {
        return pose;
    }
    public double getArmAngle() {
        return armAngle;
    }
    public void setArmAngle(double a) {
        armAngle = a;
    }

    public void setPose(Pose2d pose) {
        PoseSingleton.pose = pose;
    }
}
