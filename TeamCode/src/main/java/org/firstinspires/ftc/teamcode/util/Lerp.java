package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

// lerp for 1d travel with constant velocity
// assumes that initial and end velocity is zero
public class Lerp {
    private double totalTime;
    private double start, end;
    private ElapsedTime timer;

    // initialize the points that decide accelerations
    // all units are in PER SECOND
    public Lerp(double start, double end, double maxVel) {
        this.start = start;
        this.end = end;

        if (start == end) {
            return;
        }

        double dist = Math.abs(end - start);

        totalTime = dist / maxVel;

        timer = new ElapsedTime();
        timer.reset();
    }

    // returns the angle the pid should be targeting to achieve
    public double update() {
        if (start == end) {
            return end;
        }

        double current = timer.seconds();

        if (current > totalTime) {
            return end;
        }

        return start + (end - start) * (current / totalTime);
    }

    public double getTime() {
        return (timer != null) ? timer.seconds() : 0;
    }

    public boolean finished() {
        if (start == end) {
            return true;
        }

        return timer.seconds() > totalTime;
    }
}