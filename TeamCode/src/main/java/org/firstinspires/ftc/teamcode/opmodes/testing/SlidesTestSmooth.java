package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp

@Config
public class SlidesTestSmooth extends LinearOpMode {
    public static double pos = 0;
    public static double min = 0;
    public static double max = 25;
    public static double speed = 10;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Slides slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        double inc = 1;

        while (opModeIsActive()) {
            pos += (inc * speed) * timer.seconds();
            timer.reset();

            if (pos > max) {
                inc = -inc;
            }

            if (pos < min) {
                inc = - inc;
            }

            slides.setTarget(pos);
            slides.update(telemetry);
            telemetry.update();
        }
    }
}
