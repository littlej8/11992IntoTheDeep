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
        robot.dt.setDriveRelativeToStart(false);

        robot.schedule(
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(56, 38, 180, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(64, 38, 180, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(64, 38, -135, robot.lowerArmAction()),
                new UntilAction(robot.grabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.dropSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(56, 0, 90, robot.highBarAction()),
                robot.moveAndAction(24, 0, 90, robot.highBarAction())
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
