package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.Callable;

public class PIDFController extends PIDController{
    double Kf;
    public PIDFController(double p, double i, double d, double f) {
        super(p, i, d);
        Kf = f;
    }

    public void setKf(double kf) {
        Kf = kf;
    }

    @Override
    public double update(double current, double target) {
        return super.update(current, target) + (sgn(target - current) * (target * Kf));
    }

    public double update(double current, double target, double f) {
        return super.update(current, target) + (sgn(target - current) * (f * Kf));
    }

    private double sgn(double n) {
        return (n < 0) ? -1 : 1;
    }
}
