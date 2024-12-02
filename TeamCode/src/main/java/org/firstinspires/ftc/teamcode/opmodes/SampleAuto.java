package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;

@Autonomous(name = "Sample Auto", preselectTeleOp = "MainTeleOp")
public class SampleAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(90)), true);

        robot.schedule(
                robot.moveAndAction(16, 32, -135, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 24, 90, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(16, 32, -135, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 32, 90, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(16, 32, -135, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 32, 135, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(16, 32, -135, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(64, 24, 0, 1.0, robot.highBarAction()),
                robot.moveAndAction(64, -6, 0, 1.0, robot.highBarAction())
        );

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double finishTime = 0.0;

        while (opModeIsActive()) {
            robot.update();
            if (!robot.actionsDone())
                finishTime = timer.seconds();
            telemetry.addData("time", finishTime);
            telemetry.update();
        }
    }
}
