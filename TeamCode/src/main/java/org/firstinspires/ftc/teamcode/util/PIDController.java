package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp, Ki, Kd, prevError, sumError;
    ElapsedTime timer;

    public PIDController(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        prevError = 0;
        sumError = 0;
        timer = new ElapsedTime();
    }

    public double update(double current, double target) {
        double error = target - current;
        sumError += error * timer.seconds();
        double d = (error - prevError) / timer.seconds();

        return (error * Kp) + (sumError * Ki) + (d * Kd);
    }
}
