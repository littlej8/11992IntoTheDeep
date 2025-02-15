package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Autonomous(name = "Push Auto", preselectTeleOp = "MainTeleOp")
public class PushAutoBlue extends LinearOpMode {
    Robot robot;

    private Action noSlipAllowed() {
        return new UntilAction(new WaitAction(1000), robot.maintainPositionAction());
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(32, 64, Math.toRadians(180)));
        robot.dt.setDriveRelativeToStart(false);

        robot.schedule(
                /*robot.curveAction(-60, 4, 180),
                robot.moveAction(-60, 60, 180),
                robot.curveAction(-70, 4, 180, true),
                robot.moveAction(-70, 60, 180)
                //new UntilAction(new WaitAction(2000), robot.maintainPositionAction())*/
                //robot.moveAction(64, 60, 180),
                robot.moveAction(58, 40, 180, 0.6),
                robot.turnToAction(-60),
                robot.lowBasketAction(),
                new UntilAction(robot.moveAction(67, 55, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.claw.dropAction(), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.moveAction(58, 40, -60, 0.4), new MaintainSubsystemAction(robot.arm)),
                new UntilAction(new ActionSequence(
                        robot.moveAction(36, 40, 180),
                        robot.moveAction(36, -10, 180)
                ), robot.lowerArmAction()),
                //robot.moveAction(36, 64, 180),
                robot.moveAction(52, -10, 90),
                robot.moveAction(52, 50, 75),
                robot.moveAction(50, -10, 90),
                robot.moveAction(64, -10, 90),
                robot.moveAction(64, 48, 90),
                robot.moveAction(60, -10, 90),
                robot.moveAction(70, -10, 90),
                robot.moveAction(70, 46, 90),
                robot.moveAndAction(62, -10, 90, 1.0, robot.armAction(60)),
                new UntilAction(new WaitAction(3000), robot.moveAndAction(10, -10, 90, 1.0, new MaintainSubsystemAction(robot.arm))),
                robot.armAction(23)
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

        PoseSingleton.getInstance().setArmAngle(robot.arm.getArmPosition());
        PoseSingleton.getInstance().setPose(robot.dt.getPose());
    }
}