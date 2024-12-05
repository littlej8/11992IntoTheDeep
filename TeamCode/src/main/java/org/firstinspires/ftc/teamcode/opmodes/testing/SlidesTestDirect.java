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
public class SlidesTestDirect extends LinearOpMode {
    public static double pos = 0;
    public static double min = 0;
    public static double max = 28;
    public static double wait = 1.0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Slides slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            if (timer.seconds() > wait && pos == min) {
                pos = max;
                timer.reset();
            }

            if (timer.seconds() > wait && pos == max) {
                pos = min;
                timer.reset();
            }

            slides.setTarget(pos);
            slides.update(telemetry);
            telemetry.update();
        }
    }
}
