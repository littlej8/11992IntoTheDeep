package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;

@Autonomous(name = "Action OpMode", preselectTeleOp = "MainTeleOp")
public class SpecimenAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)), true);

        robot.schedule(
                robot.moveAndAction(26, 0, -90, 0.3, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(18, -32, -90, robot.lowerArmAction()),
                robot.moveAction(48, -32, -90),
                robot.moveAction(48, -48, 0),
                robot.moveAction(3, -48, 0, 0.8),
                robot.moveAction(-6, -36, 90),
                new UntilAction(robot.grabSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(18, 4, -90, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-6, -36, 90, robot.lowerArmAction()),
                new UntilAction(robot.grabSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(17, 8, -90, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-16, -48, -90, 1.0, robot.lowerArmAction())
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
