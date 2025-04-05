package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Disabled
@Config
@TeleOp(name="ManualControlTeleOp", group="TeleOp")
public class ManualControlTeleOp extends LinearOpMode {
    public static double turnSpeed = 0.5;
    public static double linearSpeed = 0.75;
    public static double arm_target = -40;
    public static double slides_target = 0;
    public static double claw_rot = 0;
    public static double claw_bend = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, telemetry, PoseSingleton.getInstance().getPose());
        robot.slides.setTarget(0);

        Arm.MAX_POWER = 1.0;
        Arm.LIMIT_DOWN_POWER_ABOVE_45 = false;

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        waitForStart();

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            Drivetrain.MAX_WHEEL_POWER = linearSpeed;

            // DRIVE
            double x = 0;
            if (gamepad1.left_trigger > 0.2) {
                x += gamepad1.left_trigger;
            }
            if (gamepad1.right_trigger > 0.2) {
                x -= gamepad1.right_trigger;
            }
            double y = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double turn = Math.pow(-gamepad1.right_stick_x, 3) * turnSpeed;
            robot.dt.setDrivePowers(x, y, turn);
            robot.update();

            // HANG
            /*if (gamepad1.dpad_up && gamepad1.y) {
                hardwareMap.get(DcMotorEx.class, "hang1").setPower(0.5);
                hardwareMap.get(DcMotorEx.class, "hang2").setPower(0.5);
            }*/

            // ARM
            if (gamepad2.triangle || gamepad2.y) {
                arm_target += 45 * dt;
            }

            if (gamepad2.cross || gamepad2.a) {
                arm_target -= 45 * dt;
            }

            // SLIDES
            if ((gamepad2.square) || (gamepad2.x)) {
                slides_target += 5 * dt;
            }

            if ((gamepad2.circle) || (gamepad2.b)) {
                slides_target -= 5 * dt;
            }

            // CLAW
            if (gamepad2.right_trigger > 0.1) {
                robot.claw.grip();
            }

            if (gamepad2.left_trigger > 0.1) {
                robot.claw.drop();
            }

            if (gamepad2.dpad_right) {
                claw_rot += 90 * dt;
            }

            if (gamepad2.dpad_left) {
                claw_rot -= 90 * dt;
            }

            if (gamepad2.dpad_up) {
                claw_bend += 270 * dt;
            }

            if (gamepad2.dpad_down) {
                claw_bend -= 270 * dt;
            }

            robot.arm.setTarget(arm_target);
            robot.slides.setTarget(slides_target);

            prevGamepad1.copy(gamepad1);
            prevGamepad2.copy(gamepad2);

            if (claw_rot < 0) {
                claw_rot = 0;
            }

            if (claw_rot > 180) {
                claw_rot = 180;
            }

            if (claw_bend < 0) {
                claw_bend = 0;
            }

            if (claw_bend > 270) {
                claw_bend = 270;
            }

            robot.arm.update(telemetry);
            robot.slides.update(telemetry);

            // dont break claw while arm is retracted :(
            if (arm_target > -35) {
                robot.claw.rotate(claw_rot);
                robot.claw.bend(claw_bend);
            }

            if (arm_target > 85) {
                arm_target = 85;
            }

            if (arm_target < -35) {
                arm_target = -35;
            }

            telemetry.addData("arm target", arm_target);
            telemetry.addData("slides target", slides_target);
            telemetry.addData("claw rot", claw_rot);
            telemetry.addData("claw bend", claw_bend);
            telemetry.update();
        }
    }
}
