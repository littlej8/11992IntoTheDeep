package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@Autonomous(name = "Push Auto", preselectTeleOp = "MainTeleOp")
public class PushAuto extends LinearOpMode {
    Robot robot;

    private Action noSlipAllowed() {
        return new UntilAction(new WaitAction(1000), robot.maintainPositionAction());
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(90)), true);

        robot.schedule(
                robot.moveAction(2, 24, 90),
                noSlipAllowed(),
                robot.moveAction(2, 4, 90),
                robot.moveAction(54, 4, 90),
                robot.moveAction(54, 14, 0),
                robot.moveAction(4, 14, 0),
                noSlipAllowed(),
                robot.moveAction(54, 14, 0),
                robot.moveAction(54, 22, 0),
                robot.moveAction(6, 22, 0),
                noSlipAllowed(),
                robot.moveAction(54, 22, 0),
                robot.moveAction(54, 30, 0),
                robot.moveAction(8, 30, 0)
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