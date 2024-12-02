package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp

@Config
public class SlidesTest extends LinearOpMode {
    public static double pos = 0;
    @Override
    public void runOpMode() {
        Slides slides = new Slides(hardwareMap, telemetry);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        double inc = 5;

        while (opModeIsActive()) {
            pos += inc * timer.seconds();

            if (pos > 25) {
                inc = -inc;
            }

            if (pos < 5) {
                inc = - inc;
            }

            slides.setTarget(pos);
            slides.update(telemetry);
            telemetry.update();

            timer.reset();
        }
    }
}
