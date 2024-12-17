package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Vector2d;

// wrapper for motion profile that scales x and y profiles to match
public class MotionProfile2d {
    MotionProfile x, y;

    public MotionProfile2d(Vector2d start, Vector2d end, double maxVel, double maxAccel) {
        double distX = Math.abs(end.x - start.x);
        double distY = Math.abs(end.y - start.y);
        double totalDist = Math.sqrt(distX * distX + distY * distY);

        double scaleX = distX / totalDist;
        double scaleY = distY / totalDist;

        double xMaxVel = maxVel * scaleX;
        double xMaxAccel = maxAccel * scaleX;

        double yMaxVel = maxVel * scaleY;
        double yMaxAccel = maxAccel * scaleY;

        x = new MotionProfile(start.x, end.x, xMaxVel, xMaxAccel);
        y = new MotionProfile(start.y, end.y, yMaxVel, yMaxAccel);
    }

    public Vector2d update() {
        return new Vector2d(x.update(), y.update());
    }

    public double getTime() {
        return Math.max(x.getTime(), y.getTime());
    }

    public double getTotalTime() {
        return Math.max(x.getTotalTime(), y.getTotalTime());
    }

    public boolean finished() {
        return x.finished() && y.finished();
    }
}