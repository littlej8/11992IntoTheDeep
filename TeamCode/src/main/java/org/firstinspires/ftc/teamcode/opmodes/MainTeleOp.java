package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.MecanumDrive;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            double x = -gamepad2.left_stick_y;
            double y = -gamepad2.left_stick_x;
            double rads = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double angular = gamepad2.right_stick_x;
            Vector2d linear = new Vector2d(
                Math.cos(-rads) * x - Math.sin(-rads) * y,
                Math.sin(-rads) * x + Math.cos(-rads) * y
            );

            telemetry.addData("left stick", "%.2f, %.2f", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("right stick", "%.2f, %.2f", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
            drive.setDrivePowers(new PoseVelocity2d(linear, angular));
        }
    }
}
