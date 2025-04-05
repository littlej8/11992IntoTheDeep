package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.actions.*;

@Config
@Autonomous(name = "New Specimen Auto", preselectTeleOp = "NewTeleOp")
public class NewSpecimentAuto extends LinearOpMode {
    NewRobot robot;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new NewRobot(hardwareMap, new Pose2d(-64, -8, Math.toRadians(0)), 1.0);

        robot.schedule(
            /*new UntilAction(robot.armAction(92), new WaitAction(4000)),
            telemetry1 -> {
                Arm.start_angle -= (robot.arm.getAngle() - 90);
                return false;
            },*/
            new ParallelAction(
                new ActionSequence(
                    new WaitAction(1000),
                    new UntilAction(new WaitAction(1500), robot.moveAction(-30.5, -6, 0)),
                    robot.slidesAction(10)
                ),
                robot.armAction(15),
                robot.claw.rotateAction(70),
                robot.claw.bendAction(30)
            ),
            new ParallelAction(
                robot.claw.dropAction(),
                robot.armAction(0),
                robot.moveAction(-48, -6, 0)
            ),
            new ParallelAction(
                robot.moveAction(-48, -36, 0),
                new ActionSequence(
                    new WaitAction(500),
                    new ParallelAction(
                        robot.armAction(-45),
                        robot.claw.gripAction(),
                        robot.slidesAction(0)
                    )
                )
            ),
            new ParallelAction(
                new ActionSequence(
                    robot.moveAction(-12, -36, 0),
                    robot.moveAction(-12, -50, 180),
                    robot.moveAction(-56, -50, 180),
                    robot.moveAction(-12, -45, 180),
                    robot.moveAction(-12, -60, 180),
                    robot.moveAction(-56, -60, 180),
                    robot.moveAction(-12, -55, 180)
                ),
                new MaintainSubsystemAction(robot.arm),
                new MaintainSubsystemAction(robot.slides)
            ),
            new ParallelAction(
                new ActionSequence(
                        robot.moveAction(-12, -65.5, 180),
                        robot.moveAction(-56, -65.5, 180)
                ),
                robot.armAction(-23),
                robot.claw.bendAction(50),
                robot.claw.dropAction(),
                new MaintainSubsystemAction(robot.slides)
            ),
            new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
            robot.killAction(),
            new ParallelAction(
                robot.armAction(14),
                robot.claw.rotateAction(70),
                robot.claw.bendAction(30),
                new ActionSequence(
                    robot.moveAction(-36, -4, 5),
                        new UntilAction(new WaitAction(1000), robot.moveAction(-30.5, -4, 0)),
                    robot.slidesAction(10)
                )
            ),
            new ParallelAction(
                    new ActionSequence(
                            robot.moveAndAction(-46, -38, 170, robot.armAction(0)),
                            new ParallelAction(
                                    robot.armAction(-24),
                                    new ActionSequence(
                                            new WaitAction(500),
                                            robot.moveAction(-55, -38, 180, 0.5)
                                    )
                            )
                    ),
                    robot.slidesAction(0),
                    robot.claw.bendAction(50),
                    robot.claw.dropAction()
            ),
            new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
            robot.killAction(),
            new ParallelAction(
                    robot.armAction(14),
                    robot.claw.rotateAction(70),
                    robot.claw.bendAction(30),
                    new ActionSequence(
                        robot.moveAction(-36, -4, 5),
                            new UntilAction(new WaitAction(1000), robot.moveAction(-30.5, -4, 0)),
                        robot.slidesAction(10)
                    )
            ),
            new ParallelAction(),
            new ParallelAction(
                    new ActionSequence(
                            robot.moveAndAction(-46, -38, 170, robot.armAction(0)),
                            new ParallelAction(
                                    robot.armAction(-24),
                                    new ActionSequence(
                                            new WaitAction(500),
                                            robot.moveAction(-55, -38, 180, 0.5)
                                    )
                            )
                    ),
                    robot.slidesAction(0),
                    robot.claw.bendAction(50),
                    robot.claw.dropAction()
            ),
            new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
            robot.killAction(),
            new ParallelAction(
                    robot.armAction(14),
                    robot.claw.rotateAction(70),
                    robot.claw.bendAction(30),
                    new ActionSequence(
                        robot.moveAction(-36, -4, 5),
                            new UntilAction(new WaitAction(1000), robot.moveAction(-30.5, -4, 0)),
                        robot.slidesAction(10)
                    )
            ),
            new ParallelAction(
                    new ActionSequence(
                            robot.moveAndAction(-60, -60, 0, robot.armAction(0)),
                            robot.armAction(-40, 75)
                    ),
                    robot.slidesAction(0),
                    robot.claw.bendAction(50),
                    robot.claw.dropAction()
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

        ElapsedTime time1 = new ElapsedTime();

        while (opModeInInit() && time1.seconds() < 3) {
            robot.arm.motor.setPower(-0.1);
        }

        robot.arm.motor.setPower(0);

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
