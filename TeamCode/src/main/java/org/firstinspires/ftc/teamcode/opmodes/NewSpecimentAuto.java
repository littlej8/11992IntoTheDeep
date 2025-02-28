package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.actions.*;

@Config
@Autonomous(name = "New Specimen Auto", preselectTeleOp = "MainTeleOp")
public class NewSpecimentAuto extends LinearOpMode {
    NewRobot robot;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new NewRobot(hardwareMap, new Pose2d(-64, -8, Math.toRadians(0)), 0.8);

        robot.schedule(
            new ParallelAction(
                    new ActionSequence(
                        new WaitAction(1000),
                        robot.moveAction(-30.5, -6, 0)
                    ),
                    robot.armAction(10),
                    robot.claw.rotateAction(80),
                    robot.claw.bendAction(20)
                ),
                robot.claw.dropAction(),
                new ParallelAction(
                    robot.claw.bendAction(270),
                    robot.armAction(0),
                    new ParallelAction(
                        new ActionSequence(
                            robot.moveAction(-48, -6, 0),
                            robot.moveAction(-48, -36, 0)
                        ),
                        new ActionSequence(
                            new WaitAction(500),
                            new ParallelAction(
                                robot.armAction(-50),
                                robot.claw.bendAction(270),
                                robot.claw.gripAction()
                            )
                        )
                    )
                ),
                robot.killAction(),
                robot.moveAction(-12, -36, 0),
                robot.moveAction(-12, -50, 180),
                robot.killAction(),
                robot.moveAction(-60, -50, 180),
                robot.moveAction(-12, -50, 180),
                robot.moveAction(-12, -60, 180),
                robot.killAction(),
                robot.moveAction(-60, -60, 180),
                robot.moveAction(-12, -60, 180),
                robot.moveAction(-12, -65, 180),
                robot.killAction(),
                robot.moveAction(-60, -65, 180),
                new ParallelAction(
                    new ActionSequence(
                        robot.moveAction(-46, -34, 180),
                        robot.moveAction(-55, -34, 180, 0.5)
                    ),
                    robot.armAction(-25),
                    robot.claw.bendAction(60),
                    robot.claw.dropAction()
                ),
                new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
                robot.killAction(),
                new ParallelAction(
                    robot.armAction(10),
                    robot.claw.rotateAction(80),
                    robot.claw.bendAction(20),
                    robot.moveAction(-30.5, 0, 5)
                ),
                new ParallelAction(
                        new ActionSequence(
                                robot.moveAction(-46, -34, 180),
                                robot.moveAction(-55, -34, 180, 0.5)
                        ),
                        robot.armAction(-25),
                        robot.claw.bendAction(60),
                        robot.claw.dropAction()
                ),
                new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
                robot.killAction(),
                new ParallelAction(
                        robot.armAction(10),
                        robot.claw.rotateAction(80),
                        robot.claw.bendAction(20),
                        robot.moveAction(-30.5, 0, 5)
                )
                /*
                robot.waitAction(500),
                robot.moveAction(-46, -32, 180),
                robot.moveAction(-58, -32, 180, 0.5),
                robot.waitAction(500),
                robot.moveAction(-32, 0, 5),
                robot.waitAction(500),
                robot.moveAction(-46, -32, 180),
                robot.moveAction(-58, -32, 180, 0.5),
                robot.waitAction(500),
                robot.moveAction(-32, 0, 5)*/
        );

        telemetry.clearAll();
        telemetry.addLine("Loading...");
        telemetry.update();

        robot.claw.grip();
        robot.arm.eliminateBacklash();

        telemetry.clearAll();
        telemetry.addLine("Ready to start.");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double finishTime = 0.0;

        while (opModeIsActive()) {
            robot.update(telemetry);

            if (!robot.actionsDone())
                finishTime = timer.seconds();

            telemetry.addData("time", finishTime);
            telemetry.update();
        }
    }
}
