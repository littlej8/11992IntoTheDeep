package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
@Config
public class DrivetrainOpMode extends LinearOpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = 24;
    public static double TARGET_HEADING = 180;

    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dt = new Drivetrain(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)));

        waitForStart();

        dt.moveTo(26, 0, -90);
        sleep(2000);
        dt.moveTo(20, -18, -90);
        dt.moveTo(52, -36, 90);
        dt.moveTo(3, -36, 90);
        sleep(2000);
        dt.moveTo(27, 0, -90);
        sleep(2000);
        dt.moveTo(4, -36, 90);
        sleep(2000);
        dt.moveTo(27, 0, -90);

        /*while (opModeIsActive()) {
            dt.setTargetPose(new Pose2d(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING)));

            dt.updatePose(telemetry);
            dt.updateMovement(telemetry);
            dt.debug(telemetry);
            telemetry.update();
        }*/
    }
}
