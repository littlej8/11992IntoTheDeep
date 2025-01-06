package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        Robot robot = new Robot(hardwareMap, telemetry, PoseSingleton.getInstance().getPose(), 420);
        robot.slides.setTarget(0);
        robot.arm.setTarget(0);
        Drivetrain.MAX_WHEEL_POWER = 0.75;

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        waitForStart();

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);

        while (opModeIsActive()) {
            Drivetrain.MAX_WHEEL_POWER = linearSpeed;

            // DRIVE
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

            // ARM
            if (gamepad2.triangle && !prevGamepad2.triangle) {
                robot.arm.setTarget(90);
            }

            if (gamepad2.cross && !prevGamepad2.cross) {
                robot.arm.setTarget(0);
            }

            if (gamepad2.dpad_down && !prevGamepad2.dpad_down) {
                robot.arm.setTarget(-40);
            }

            if (gamepad2.dpad_up && !prevGamepad2.dpad_up) {
                robot.arm.setTarget(200);
            }

            // SLIDES
            if (gamepad2.square && !prevGamepad2.square) {
                robot.slides.setTarget(25);
            }

            if (gamepad2.circle && !prevGamepad2.circle) {
                robot.slides.setTarget(0);
            }

            prevGamepad1.copy(gamepad1);
            prevGamepad2.copy(gamepad2);

            robot.arm.update(telemetry);
            robot.slides.update(telemetry);

            //telemetry.addData("slides current", robot.slides.);
            //telemetry.addData("left stick", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            //telemetry.addData("right stick", "%.2f, %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("triangle", gamepad2.triangle);
            telemetry.addData("prev triangle", prevGamepad2.triangle);
            telemetry.update();
        }
    }
}
