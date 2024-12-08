package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@Autonomous(name = "Push Auto Blue", preselectTeleOp = "MainTeleOp")
public class PushAutoBlue extends LinearOpMode {
    Robot robot;

    private Action noSlipAllowed() {
        return new UntilAction(new WaitAction(1000), robot.maintainPositionAction());
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(32, 64, Math.toRadians(180)), true);

        robot.schedule(
                robot.moveAction(24, 2, 180),
                noSlipAllowed(),
                robot.moveAction(4, 2, 180),
                robot.moveAction(4, 64, 180),
                robot.moveAction(18, 64, 90),
                robot.moveAction(24, 6, 75),
                noSlipAllowed(),
                robot.moveAction(18, 64, 90),
                robot.moveAction(28, 64, 90),
                robot.moveAction(28, 6, 90),
                noSlipAllowed(),
                robot.moveAction(28, 64, 90),
                robot.moveAction(40, 64, 90),
                robot.moveAction(40, 8, 90)
                /*robot.moveAction(42, 24, 0),
                robot.moveAction(-84, 24, 0),
                robot.moveAction(-100, 0, 0)*/
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