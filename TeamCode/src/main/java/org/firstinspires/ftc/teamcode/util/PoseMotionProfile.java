package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// wrapper for MotionProfile2d for linear movement and Lerp for angular
// scales both so they reach goals at the same time
public class PoseMotionProfile {
    MotionProfile2d profile;
    Lerp lerp;

    // assumes heading in radians
    public PoseMotionProfile(Pose2d start, Pose2d end, double maxVel, double maxAccel) {
        profile = new MotionProfile2d(start.position, end.position, maxVel, maxAccel);
        double headingVel = Math.abs(end.heading.toDouble() - start.heading.toDouble()) / profile.getTotalTime();
        lerp = new Lerp(start.heading.toDouble(), end.heading.toDouble(), headingVel);
    }

    public Pose2d update() {
        return new Pose2d(profile.update(), lerp.update());
    }

    public double getTime() {
        return profile.getTime();
    }

    public double getTotalTime() {
        return profile.getTotalTime();
    }

    public boolean finished() {
        return profile.finished() && lerp.finished();
    }
}
