package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.shuttle.HardwareTaskScope;

@TeleOp
@Config
public class DrivetrainOpMode extends LinearOpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = 24;
    public static double TARGET_HEADING = 180;

    Drivetrain dt;

    void hookSpecimen(double fudgeX, double fudgeY) {
        dt.moveTo(26 + fudgeX, 0 + fudgeX, -90);
        dt.maintainPosition(2000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dt = new Drivetrain(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)));

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        /* ***ONLY USE WHEN THE TASKS OPERATE ON SEPARATE ACTUATORS
           TO AVOID UNDEFINED BEHAVIOR***
        try (var scope = HardwareTaskScope.open()) {
            scope.fork(() -> dt.moveToWithSpeed(26, 0, -90, 0.7));
            scope.fork(() -> arm.goToPosition(arm.Position.HIGH_HOOK_POS));
            scope.fork(() -> slides.goToPosition(slides.Position.HIGH_HOOK_POS));

            scope.join();
        }
        */

        // 1 tile == 24 inches
        // +x == towards other team
        // +y == towards basket
        // +h == left (counter-clockwise)
        dt.moveToWithSpeed(26, 0, -90, 0.3);
        dt.maintainPosition(2000);
        dt.moveTo(18, -32, -90);
        dt.moveToWithSpeed(48, -32, -90, 0.5);
        dt.moveTo(48, -48, 0);
        //dt.maintainPosition(500);
        dt.moveToWithSpeed(3, -48, 0, 0.8);
        dt.moveTo(-6, -36, 90);
        dt.maintainPosition(2000);
        //dt.moveTo(3, 2, -90);
        dt.moveTo(18, 4, -90);
        dt.maintainPosition(2000);
        dt.moveTo(-6, -36, 90);
        dt.maintainPosition(2000);
        dt.moveTo(17, 8, -90);
        dt.maintainPosition(2000);
        dt.moveToWithSpeed(-10, -36, -90, 0.8);

        double time = timer.seconds();

        while (opModeIsActive()) {
            /*dt.setTargetPose(new Pose2d(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING)));

            dt.updatePose(telemetry);
            dt.updateMovement(telemetry);
            dt.debug(telemetry);*/
            telemetry.addData("total time", time);
            telemetry.update();
        }
    }
}
