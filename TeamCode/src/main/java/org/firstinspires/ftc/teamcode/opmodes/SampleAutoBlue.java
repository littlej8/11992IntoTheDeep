package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Autonomous(name = "Sample Auto Blue", preselectTeleOp = "MainTeleOp")
public class SampleAutoBlue extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(32, 64, Math.toRadians(180)), true);

        robot.schedule(
                robot.moveAndAction(32, 16, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(24, 32, 180, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 16, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 32, 180, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 16, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 32, -135, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 16, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(24, 64, 90, robot.highBarAction()),
                robot.moveAndAction(-6, 64, 90, robot.highBarAction())
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

        PoseSingleton.getInstance().setPose(robot.dt.getPose());
    }
}
