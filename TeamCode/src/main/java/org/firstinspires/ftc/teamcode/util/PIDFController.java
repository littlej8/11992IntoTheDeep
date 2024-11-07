package org.firstinspires.ftc.teamcode.util;

public class PIDFController extends PIDController{
    double Kf;
    public PIDFController(double p, double i, double d, double f) {
        super(p, i, d);
        Kf = f;
    }

    @Override
    public double update(double current, double target) {
        return super.update(current, target) + (target * Kf);
    }
}
