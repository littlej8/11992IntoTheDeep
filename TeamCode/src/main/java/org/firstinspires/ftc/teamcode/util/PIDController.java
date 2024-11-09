package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp, Ki, Kd, prevError, sumError;
    boolean wrap = false;
    ElapsedTime timer;

    public PIDController(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        prevError = 0;
        sumError = 0;
        timer = new ElapsedTime();
    }

    public PIDController(double p, double i, double d, boolean angleWrap) {
        this(p, i, d);
        wrap = true;
    }

    public void setKp(double kp) {
        Kp = kp;
    }

    public void setKi(double ki) {
        Ki = ki;
    }

    public void setKd(double kd) {
        Kd = kd;
    }

    public void setPID(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public double update(double current, double target) {
        double error = target - current;

        if (wrap) {
            while (error > Math.PI)
                error -= 2 * Math.PI;
            while (error < -Math.PI)
                error += 2 * Math.PI;
        }

        sumError += error * timer.seconds();
        double d = (error - prevError) / timer.seconds();

        return (error * Kp) + (sumError * Ki) + (d * Kd);
    }
}
