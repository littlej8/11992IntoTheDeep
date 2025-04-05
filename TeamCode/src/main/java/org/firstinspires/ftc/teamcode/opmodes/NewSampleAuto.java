package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.actions.*;

@Config
@Autonomous(name = "New Sample Auto", preselectTeleOp = "NewTeleOp")
public class NewSampleAuto extends LinearOpMode {
    NewRobot robot;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new NewRobot(hardwareMap, new Pose2d(-64, 32, Math.toRadians(0)), 1.0);

        robot.schedule(
            robot.moveAndAction(-52, 52, -45, robot.armAction(80)),
            new UntilAction(new WaitAction(500), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(robot.slidesAction(28), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(
                new ActionSequence(
                    new ParallelAction(new WaitAction(1000), robot.moveAction(-52, 56, -45), robot.claw.bendAction(30), robot.claw.rotateAction(80)),
                    robot.claw.dropAction(),
                    new ParallelAction(robot.moveAction(-48, 48, -45), robot.claw.bendAction(140), robot.armAction(60))
                ),
                new ParallelAction(new MaintainSubsystemAction(robot.dt), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
            ),
            new ParallelAction(robot.slidesAction(0), new MaintainSubsystemAction(robot.arm)),
            new ParallelAction(
                robot.moveAction(-40, 50, 0),
                new ActionSequence(
                    robot.armAction(-25, 75),
                    new UntilAction(robot.slidesAction(12.5), new MaintainSubsystemAction(robot.arm)),
                    new ParallelAction(new WaitAction(1000), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides)),
                    new UntilAction(robot.armAction(-40, 75), new MaintainSubsystemAction(robot.slides)),
                    new ParallelAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
                ),
                new MaintainSubsystemAction(robot.dt)
            ),
            robot.moveAndAction(-52, 52, -45, robot.armAction(80)),
            new UntilAction(new WaitAction(500), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(robot.slidesAction(28), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(
                    new ActionSequence(
                            new ParallelAction(new WaitAction(1000), robot.moveAction(-52, 56, -45), robot.claw.bendAction(30), robot.claw.rotateAction(80)),
                            robot.claw.dropAction(),
                            new ParallelAction(robot.moveAction(-48, 48, -45), robot.claw.bendAction(140), robot.armAction(60))
                    ),
                    new ParallelAction(new MaintainSubsystemAction(robot.dt), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
            ),
            new ParallelAction(robot.slidesAction(0), new MaintainSubsystemAction(robot.arm)),
            new ParallelAction(
                    robot.moveAction(-40, 61, 0),
                    new ActionSequence(
                            robot.armAction(-25, 75),
                            new UntilAction(robot.slidesAction(12.5), new MaintainSubsystemAction(robot.arm)),
                            new ParallelAction(new WaitAction(1000), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides)),
                            new UntilAction(robot.armAction(-40, 75), new MaintainSubsystemAction(robot.slides)),
                            new ParallelAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
                    ),
                    new MaintainSubsystemAction(robot.dt)
            ),
            robot.moveAndAction(-52, 52, -45, robot.armAction(80)),
            new UntilAction(new WaitAction(500), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(robot.slidesAction(28), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(
                    new ActionSequence(
                            new ParallelAction(new WaitAction(1000), robot.moveAction(-52, 56, -45), robot.claw.bendAction(30), robot.claw.rotateAction(80)),
                            robot.claw.dropAction(),
                            new ParallelAction(robot.moveAction(-48, 48, -45), robot.claw.bendAction(140), robot.armAction(60))
                    ),
                    new ParallelAction(new MaintainSubsystemAction(robot.dt), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
            ),
            new ParallelAction(robot.slidesAction(0), new MaintainSubsystemAction(robot.arm)),
            new ParallelAction(
                    robot.moveAction(-27, 54, 90),
                    robot.claw.rotateAction(170),
                    new ActionSequence(
                            robot.armAction(-25, 75),
                            new UntilAction(robot.slidesAction(13), new MaintainSubsystemAction(robot.arm)),
                            new ParallelAction(new WaitAction(1500), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides)),
                            new UntilAction(robot.armAction(-40, 75), new MaintainSubsystemAction(robot.slides)),
                            new ParallelAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
                    ),
                    new MaintainSubsystemAction(robot.dt)
            ),
            robot.moveAndAction(-52, 52, -45, robot.armAction(80)),
            new UntilAction(new WaitAction(500), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(robot.slidesAction(28), new MaintainSubsystemAction(robot.arm)),
            new UntilAction(
                    new ActionSequence(
                            new ParallelAction(new WaitAction(1000), robot.moveAction(-52, 56, -45), robot.claw.bendAction(30), robot.claw.rotateAction(80), new MaintainSubsystemAction(robot.arm)),
                            new ParallelAction(robot.claw.dropAction(), new MaintainSubsystemAction(robot.arm)),
                            new ParallelAction(robot.claw.bendAction(140), robot.armAction(60), new MaintainSubsystemAction(robot.arm))
                    ),
                    new ParallelAction(new MaintainSubsystemAction(robot.dt), new MaintainSubsystemAction(robot.arm), new MaintainSubsystemAction(robot.slides))
            ),
            new ParallelAction(robot.slidesAction(0), robot.armAction(-40, 75)),
            /*robot.moveAndAction(-8, 40, -90, new MaintainSubsystemAction(robot.arm)),
            robot.moveAndAction(-8, 22, -90, new MaintainSubsystemAction(robot.arm)),*/
            robot.killAction()
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
