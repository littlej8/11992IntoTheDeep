package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.NewRobot;

@TeleOp(name="Drivetrain Testing", group="Dashboard Tests")
public class NewDrivetrainTuning extends LinearOpMode {
    NewRobot robot;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new NewRobot(hardwareMap, new Pose2d(-48, -48, Math.toRadians(0)), 1.0);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            // MOVEMENT TUNING
            /*if (timer.seconds() < 3) {
                robot.dt.setTarget(new Pose2d(48, 0, 0));
            } else if (timer.seconds() < 6) {
                robot.dt.setTarget(new Pose2d(0, 0, 0));
            } else {
                timer.reset();
            }*/
            // HEADING TUNING
            if (timer.seconds() < 5) {
                robot.dt.setTarget(new Pose2d(48, -48, Math.toRadians(90)));
            } else if (timer.seconds() < 10) {
                robot.dt.setTarget(new Pose2d(48, 48, Math.toRadians(180)));
            } else if (timer.seconds() < 15) {
                robot.dt.setTarget(new Pose2d(-48, 48, Math.toRadians(270)));
            } else if (timer.seconds() < 20) {
                robot.dt.setTarget(new Pose2d(-48, -48, Math.toRadians(0)));
            } else {
                timer.reset();
            }
            robot.update(telemetry);
            telemetry.update();
        }
    }
}
