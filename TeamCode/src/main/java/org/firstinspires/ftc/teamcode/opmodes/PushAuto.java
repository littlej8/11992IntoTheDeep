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
                robot.moveAction(64, 4, 90),
                robot.moveAction(64, 18, 0),
                robot.moveAction(6, 24, -15),
                noSlipAllowed(),
                robot.moveAction(64, 18, 0),
                robot.moveAction(64, 28, 0),
                robot.moveAction(6, 28, 0),
                noSlipAllowed(),
                robot.moveAction(64, 28, 0),
                robot.moveAction(64, 40, 0),
                robot.moveAction(8, 40, 0),
                robot.moveAction(24, 42, 0),
                robot.moveAction(24, -84, 0),
                robot.moveAction(0, -100, 0)
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