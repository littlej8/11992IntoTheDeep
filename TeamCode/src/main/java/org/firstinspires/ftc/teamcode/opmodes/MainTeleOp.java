package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Mat;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)), true);
        Drivetrain.MAX_WHEEL_POWER = 0.75;

        waitForStart();

        while (opModeIsActive()) {
            double x = 0;
            if (gamepad1.left_trigger > 0) {
                x += gamepad1.left_trigger;
            }
            if (gamepad1.right_trigger > 0) {
                x += -gamepad1.right_trigger;
            }
            double y = gamepad1.left_stick_y;
            double turn = Math.pow(-gamepad1.right_stick_x, 3);
            robot.dt.setDrivePowers(x, y, turn);

            telemetry.addData("left stick", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("right stick", "%.2f, %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.update();
        }
        /*MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.lazyImu.get().resetYaw();

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
        }*/
    }
}
