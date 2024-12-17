package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

// motion profile for 1d travel
// assumes that initial and end velocity is zero
public class MotionProfile {
    private double totalTime, accelTime, cruiseTime, decelTime;
    private double start, end;
    private double maxVel, maxAccel;
    private double dirMult = 1;
    private ElapsedTime timer;

    // initialize the points that decide accelerations
    // all units are in PER SECOND
    public MotionProfile(double start, double end, double maxVel, double maxAccel) {
        this.start = start;
        this.end = end;

        if (start == end) {
            return;
        }

        if (end < start) {
            dirMult = -1;
        }

        double dist = Math.abs(end - start);

        accelTime = maxVel / maxAccel;

        double halfDist = dist / 2;
        double accelDist = 0.5 * maxAccel * accelTime * accelTime; // d = 1/2vi (zero) + 1/2at^2

        // can't reach max vel in time
        if (accelDist > halfDist) {
            accelTime = Math.sqrt(halfDist / (0.5 * maxAccel));
        }

        // recalculate in case time changed
        accelDist = 0.5 * maxAccel * accelTime * accelTime;
        maxVel = maxAccel * accelTime;

        double cruiseDist = dist - (2 * accelDist); // cruise for the distance that we aren't accelerating
        cruiseTime = cruiseDist / maxVel;

        if (accelDist > halfDist) {
            cruiseTime = 0;
        }

        decelTime = accelTime; // decel at same rate as accel

        totalTime = accelTime + cruiseTime + decelTime;

        timer = new ElapsedTime();
        timer.reset();

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    // returns the position the pid should be targeting to achieve
    // around the desired acceleration and velocity
    public double update() {
        if (start == end) {
            return end;
        }

        double decelStart = accelTime + cruiseTime;
        double current = timer.seconds();

        // finished
        if (current > totalTime) {
            return end;
        }

        // accelerating
        if (current < accelTime) {
            return start + (0.5 * maxAccel * current * current) * dirMult;
        }

        // crusing
        if (current < decelStart) {
            double accelDist = (0.5 * maxAccel * accelTime * accelTime);
            double cruiseDist = (current - accelTime) * maxVel;

            return start + (accelDist + cruiseDist) * dirMult;
        }

        // else decel
        double accelDist = (0.5 * maxAccel * accelTime * accelTime);
        double cruiseDist = cruiseTime * maxVel;

        double currentDecelTime = current - decelStart;
        double decelDist = (maxVel * currentDecelTime) - (0.5 * maxAccel * currentDecelTime * currentDecelTime);

        return start + (accelDist + cruiseDist + decelDist) * dirMult;
    }

    public double getTime() {
        return (timer != null) ? timer.seconds() : 0;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public boolean finished() {
        if (start == end) {
            return true;
        }

        return timer.seconds() > totalTime;
    }
}