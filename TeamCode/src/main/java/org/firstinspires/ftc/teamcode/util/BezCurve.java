package org.firstinspires.ftc.teamcode.util;

import java.util.List;
import java.util.ArrayList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

// https://www.desmos.com/calculator/wbvbroge6x

public class BezCurve {
    Vector2d[] points;
    double maxVel, maxAccel;
    double totalTime;
    ElapsedTime timer;

    public BezCurve(Vector2d[] points, double maxVel, double maxAccel) {
        this.points = points;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        calculateTime();

        timer = new ElapsedTime();
        timer.reset();
    }

    public BezCurve(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3, double maxVel, double maxAccel) {
        this(new Vector2d[]{p0, p1, p2, p3}, maxVel, maxAccel);
    }

    public double getTotalTime() {
        return totalTime;
    }

    public boolean finished() {
        return timer.seconds() >= totalTime;
    }

    public void draw(Canvas c) {
        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);

        for (int i = 1; i < 100; i++) {
            Vector2d prev = getPoint((i-1) / 100.0);
            Vector2d cur = getPoint(i / 100.0);
            c.strokeLine(prev.x, prev.y, cur.x, cur.y);
        }
    }

    public Vector2d update() {
        double cur = timer.seconds();

        if (cur > totalTime) {
            return points[points.length-1];
        }

        return getPoint(cur / totalTime);
    }

    private void calculateTime() {
        int segments = 100;
        List<Vector2d> p = new ArrayList<>();
        List<Vector2d> v = new ArrayList<>();

        for (int i = 0; i < segments; i++) {
            double t = i / (double) segments;
            p.add(getPoint(t));
            v.add(getVel(t));
        }

        totalTime = 0;
        double prevVel = 0;

        for (int i = 1; i < p.size(); i++) {
            Vector2d cur = p.get(i);
            Vector2d prev = p.get(i - 1);
            Vector2d velVec = v.get(i);

            double dist = Math.hypot(cur.x - prev.x, cur.y - prev.y);
            double vel = Math.min(maxVel, Math.hypot(velVec.x, velVec.y));
            double accelTime = Math.abs(vel - prevVel) / maxAccel;
            double segmentTime = Math.max(accelTime, dist / vel);

            totalTime += segmentTime;
            prevVel = vel;
        }
    }

    private Vector2d getPoint(double t) {
        double px = 0, py = 0;
        int n = points.length - 1;

        for (int i = 0; i <= n; i++) {
            double c = getCoeff(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
            px += c * points[i].x;
            py += c * points[i].y;
        }

        return new Vector2d(px, py);
    }

    private Vector2d getVel(double t) {
        double velx = 0, vely = 0;
        int n = points.length - 1;

        for (int i = 0; i < n; i++) {
            double c = getCoeff(n - 1, i) * Math.pow(1 - t, n - 1 - i) * Math.pow(t, i);
            velx += c * n * (points[i+1].x - points[i].x);
            vely += c * n * (points[i+1].y - points[i].y);
        }

        return new Vector2d(velx, vely);
    }

    private int getCoeff(int n, int k) {
        int ret = 1;
        for (int i = 1; i <= k; i++) {
            ret *= (n - (k - i));
            ret /= i;
        }
        return ret;
    }
}