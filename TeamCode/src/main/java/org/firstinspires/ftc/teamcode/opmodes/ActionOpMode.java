package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;

@TeleOp
public class ActionOpMode extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)), true);

        robot.schedule(
                new MaintainAction(robot.moveAction(26, 0, -90), robot.highHookAction()),
                robot.moveAction(-10, -36, -90)
        );

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
