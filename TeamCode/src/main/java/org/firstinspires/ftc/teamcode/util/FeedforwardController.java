package org.firstinspires.ftc.teamcode.util;

public class FeedforwardController {
    double Ks, Kv, Ka;

    public FeedforwardController(double s, double v, double a) {
        Ks = s;
        Kv = v;
        Ka = a;
    }

    public double update(double targetVel, double targetAccel) {
        return Ks + (targetVel * Kv) + (targetAccel * Ka);
    }
}
