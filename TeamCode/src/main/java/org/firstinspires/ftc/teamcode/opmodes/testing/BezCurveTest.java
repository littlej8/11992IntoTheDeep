package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.BezCurve;

@Disabled
@Autonomous
public class BezCurveTest extends LinearOpMode {
    public void runOpMode() {
        Drivetrain dt = new Drivetrain(hardwareMap, telemetry, new Pose2d(-17, 64, Math.toRadians(180)));
        Drivetrain.MAX_WHEEL_POWER = 0.75;
        dt.setDriveRelativeToStart(false);

        Vector2d start = dt.getPose().position;
        Vector2d end = new Vector2d(-60, 4);
        //Vector2d[] points = new Vector2d[]{new Vector2d(-8, 64), new Vector2d(-75, 38.2), new Vector2d(-17.5, 21.9), new Vector2d(-60, 4)};
        Vector2d[] points = new Vector2d[]{start, new Vector2d(end.x, start.y), new Vector2d(start.x, end.y), end};

        waitForStart();

        BezCurve mp = new BezCurve(points, 24, 24);

        while (opModeIsActive()) {
            dt.setTargetPose(new Pose2d(mp.update(), Math.toRadians(180)));
            dt.updatePose(telemetry);
            dt.updateMovement(telemetry);

            TelemetryPacket p = new TelemetryPacket(true);
            Canvas c = p.fieldOverlay();
            mp.draw(c);
            dt.draw(c);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.update();
        }
    }
}
