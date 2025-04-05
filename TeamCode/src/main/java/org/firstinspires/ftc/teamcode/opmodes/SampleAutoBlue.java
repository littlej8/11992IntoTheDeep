package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Disabled
@Autonomous(name = "Sample Auto", preselectTeleOp = "MainTeleOp")
public class SampleAutoBlue extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(32, 64, Math.toRadians(180)));
        robot.dt.setDriveRelativeToStart(false);

        robot.schedule(
                robot.moveAction(58, 40, 180, 0.6),
                robot.turnToAction(-60),
                robot.lowBasketAction(),
                new UntilAction(robot.moveAction(66, 54, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.claw.dropAction(), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.moveAction(58, 40, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.armToGrabSampleAction(), robot.moveAction(56, 15, 180)),
                new UntilAction(new WaitAction(2000), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
                new ParallelAction(robot.armAction(-10), new ActionSequence(
                        new WaitAction(500),
                        robot.moveAction(58, 40, 180)
                )),
                new UntilAction(robot.turnToAction(-60), new MaintainSubsystemAction(robot.arm)),
                robot.lowBasketAction(),
                new UntilAction(robot.moveAction(68, 50, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.claw.dropAction(), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.moveAction(58, 40, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.armToGrabSampleAction(), robot.moveAction(72, 15, 180))/*
                new UntilAction(robot.moveAction(64, 48, -60, 0.3), new ParallelAction(
                        new MaintainSubsystemAction(robot.arm),
                        new MaintainSubsystemAction(robot.slides)
                )),
                robot.retractFromBasketAction(),
                robot.moveAndAction(54, 48, -45, new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.armToGrabSampleAction(), robot.turnToAction(180)),
                robot.moveAndAction(56, 38, 180, new MaintainSubsystemAction(robot.arm))*/
                /*new UntilAction(robot.armToGrabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.claw.dropAction(), new ParallelAction(
                        new MaintainSubsystemAction(robot.arm),
                        new MaintainSubsystemAction(robot.slides)
                )),
                robot.moveAndAction(64, 38, 180, robot.lowerArmAction()),
                new UntilAction(robot.armToGrabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.claw.dropAction(), new ParallelAction(
                        new MaintainSubsystemAction(robot.arm),
                        new MaintainSubsystemAction(robot.slides)
                )),
                robot.moveAndAction(64, 38, -135, robot.lowerArmAction()),
                new UntilAction(robot.armToGrabSampleAction(), robot.maintainPositionAction()),
                robot.moveAndAction(58, 52, -45, robot.highBasketAction()),
                new UntilAction(robot.claw.dropAction(), new ParallelAction(
                        new MaintainSubsystemAction(robot.arm),
                        new MaintainSubsystemAction(robot.slides)
                )),
                robot.moveAndAction(56, 0, 90, robot.prepareToHookAction()),
                robot.moveAndAction(24, 0, 90, robot.prepareToHookAction())*/
        );

        robot.claw.grip();

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
