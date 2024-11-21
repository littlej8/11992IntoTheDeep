package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;

@Autonomous(name = "Action OpMode", preselectTeleOp = "MainTeleOp")
public class ActionOpMode extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)), true);

        robot.schedule(
                new UntilAction(robot.moveAction(26, 0, -90), robot.highBarAction()),
                robot.hookSpecimenAction(),
                robot.moveAction(-10, -36, -90)
        );

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
