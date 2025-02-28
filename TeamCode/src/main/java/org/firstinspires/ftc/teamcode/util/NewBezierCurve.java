package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Vector2d;

public class NewBezierCurve {
    private final Vector2d[] points = new Vector2d[4];

    public NewBezierCurve(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        points[0] = new Vector2d(x1, y1);
        points[1] = new Vector2d(x2, y2);
        points[2] = new Vector2d(x3, y3);
        points[3] = new Vector2d(x4, y4);
    }

    private Vector2d lerp(Vector2d p1, Vector2d p2, double t) {
        return new Vector2d(
                p1.x + ((p2.x - p1.x) * t),
                p1.y + ((p2.y - p1.y) * t)
        );
    }

    // 0 < t < 1
    public Vector2d getPoint(double t) {
        Vector2d p01 = lerp(points[0], points[1], t);
        Vector2d p12 = lerp(points[1], points[2], t);
        Vector2d p23 = lerp(points[2], points[3], t);

        Vector2d p0112 = lerp(p01, p12, t);
        Vector2d p1223 = lerp(p12, p23, t);

        return lerp(p0112, p1223, t);
    }

    public Vector2d getVelocity(double t) {
        Vector2d p01 = lerp(points[0], points[1], t);
        Vector2d p12 = lerp(points[1], points[2], t);
        Vector2d p23 = lerp(points[2], points[3], t);

        Vector2d p0112 = lerp(p01, p12, t);
        Vector2d p1223 = lerp(p12, p23, t);

        return new Vector2d(p1223.x - p0112.x, p1223.y - p0112.y);
    }

    private double dist(Vector2d v1, Vector2d v2) {
        return Math.sqrt(
                Math.pow(v2.x - v1.x, 2) + Math.pow(v2.y - v1.y, 2)
        );
    }

    public double closestPoint(Vector2d pos) {
        double max = 0;
        for (int i = 1; i <= 100; i++) {
            double t = (double)i / 100.0;
            if (dist(getPoint(t), pos) < dist(getPoint(max), pos)) {
                max = t;
            }
        }
        return max;
    }
}
