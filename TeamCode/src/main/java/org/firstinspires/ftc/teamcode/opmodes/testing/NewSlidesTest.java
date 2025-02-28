package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.NewRobot;

@Config
@TeleOp
public class NewSlidesTest extends LinearOpMode {
    NewRobot robot;

    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new NewRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.slides.setTarget(target);
            robot.slides.update(telemetry);
            telemetry.update();
        }
    }
}
