package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Config
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    public static double turnSpeed = 0.75;
    public static double linearSpeed = 1.0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, telemetry, PoseSingleton.getInstance().getPose(), true);
        Drivetrain.MAX_WHEEL_POWER = 0.75;

        waitForStart();

        while (opModeIsActive()) {
            Drivetrain.MAX_WHEEL_POWER = linearSpeed;

            double x = 0;
            if (gamepad1.left_trigger > 0.2) {
                x += gamepad1.left_trigger;
            }
            if (gamepad1.right_trigger > 0.2) {
                x -= gamepad1.right_trigger;
            }
            double y = gamepad1.left_stick_y;
            double turn = Math.pow(-gamepad1.right_stick_x, 3) * turnSpeed;
            robot.dt.setDrivePowers(x, y, turn);
            robot.updateWithVision(telemetry);

            telemetry.addData("left stick", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("right stick", "%.2f, %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
