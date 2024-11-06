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
            double x = Math.pow(-gamepad2.left_stick_y, 3);
            //x = (Math.abs(x) < 0.2) ? 0 : x;
            double y = Math.pow(-gamepad2.left_stick_x, 3);
            //y = (Math.abs(y) < 0.2) ? 0 : y;
            double rads = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double angular = Math.pow(gamepad2.right_stick_x, 3);
            //angular = (Math.abs(angular) < 0.2) ? 0 : angular;
            Vector2d linear = new Vector2d(
                Math.cos(-rads) * x - Math.sin(-rads) * y,
                Math.sin(-rads) * x + Math.cos(-rads) * y
            );

            telemetry.addData("front encoders", "%d, %d", drive.leftFront.getCurrentPosition(), drive.rightFront.getCurrentPosition());
            telemetry.addData("back encoders", "%d, %d", drive.leftBack.getCurrentPosition(), drive.rightBack.getCurrentPosition());
            telemetry.addData("left stick", "%.2f, %.2f", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("right stick", "%.2f, %.2f", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
            drive.setDrivePowers(new PoseVelocity2d(linear, angular));
        }
    }
}
