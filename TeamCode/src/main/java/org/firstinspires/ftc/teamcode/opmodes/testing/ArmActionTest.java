package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@Config
@Autonomous(name = "Arm Action Test", preselectTeleOp = "MainTeleOp")
public class ArmActionTest extends LinearOpMode {
    Robot robot;
    public static double arm_testing_target = -40;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(-8, 64, Math.toRadians(180)));

        robot.schedule(
                robot.highBasketAction(),
                new UntilAction(new WaitAction(5000), new ParallelAction(
                        new MaintainSubsystemAction(robot.arm),
                        new MaintainSubsystemAction(robot.slides)
                ))
        );

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double finishTime = 0.0;

        while (opModeIsActive()) {
            //robot.arm.setTarget(arm_testing_target);
            robot.update(telemetry);
            telemetry.update();
        }
    }
}
