package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

/*
    Driver 1:
    - left stick forward/backward
    - right stick turn
    - left/right trigger strafe left/right

    Driver 2:
    - right/left trigger to grip/drop claw
    - dpad up/down to adjust arm for the current state
    - cross to retract arm
    - circle to grab spec from wall
      - circle again while in this state to grab from sub
        - while in this mode dpad up/down makes ik target move vertically
        - also left stick forward/backward moves ik target forward/backward
        - also dpad left/right to rotate claw to match samples
    - square to hook spec
    - triangle to drop in basket
 */

@Config
@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    public static String arm_state = "retracted";
    boolean switching_states = false;

    void setState(String s) {
        arm_state = s;
        switching_states = true;
    }

    public static double turnSpeed = 0.5;
    public static double linearSpeed = 0.75;
    public static double arm_target = -40;
    public static double slides_target = 0;
    public static double claw_rot = 0;
    public static double claw_bend = 0;

    // inches
    public static double arm_ik_target_x = 0;
    public static double arm_ik_target_y = 6;
    public static double arm_height = 240 / 25.4;
    public static double arm_length = 384 / 25.4;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, telemetry, PoseSingleton.getInstance().getPose());
        robot.slides.setTarget(0);
        Drivetrain.MAX_WHEEL_POWER = 0.75;

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
                x += gamepad1.left_trigger / 2;
            }
            if (gamepad1.right_trigger > 0.2) {
                x -= gamepad1.right_trigger / 2;
            }
            double y = gamepad1.left_stick_y;
            double turn = Math.pow(-gamepad1.right_stick_x, 3) * turnSpeed;
            robot.dt.setDrivePowers(x, y, turn);
            robot.updateWithVision(telemetry);

            if (gamepad2.right_trigger > 0.1) {
                robot.claw.grip();
            }

            if (gamepad2.left_trigger > 0.1) {
                robot.claw.drop();
            }

            if (gamepad2.cross && !prevGamepad2.cross) {
                setState("retracted");
            }

            if (gamepad2.circle && !prevGamepad2.circle && arm_state.equals("grabbing_wall")) {
                setState("grabbing_sub");
            }

            if (gamepad2.circle && !prevGamepad2.circle) {
                setState("grabbing_wall");
            }

            if (gamepad2.square && !prevGamepad2.square) {
                setState("hooking");
            }

            if (gamepad2.triangle && !prevGamepad2.triangle) {
                setState("basket");
            }

            if (switching_states) {
                switching_states = false;
                switch (arm_state) {
                    case "retracted": {
                        arm_target = -35;
                        slides_target = 0;
                        claw_bend = 0;
                        claw_rot = 0;
                        break;
                    }
                    case "grabbing_wall": {
                        arm_target = -17;
                        slides_target = 0;
                        claw_bend = 240;
                        claw_rot = 90;
                        robot.claw.drop();
                        break;
                    }
                    case "grabbing_sub": {
                        claw_rot = 90;
                        arm_ik_target_x = 0;
                        arm_ik_target_y = 4;
                        robot.claw.drop();
                        break;
                    }
                    case "hooking": {
                        arm_target = 19;
                        slides_target = 0;
                        claw_bend = 270;
                        claw_rot = 90;
                        break;
                    }
                    case "basket": {
                        arm_target = 90;
                        slides_target = 25;
                        claw_bend = 270;
                        claw_rot = 90;
                        break;
                    }
                    default: {
                        arm_state = "retracted";
                        switching_states = true;
                        break;
                    }
                }
            }

            if (arm_state.equals("grabbing_sub")) {
                arm_target = Math.atan2(-arm_height + arm_ik_target_y, arm_ik_target_x + arm_length);
                slides_target = Math.hypot(arm_ik_target_x + arm_length, arm_height);
                claw_bend = 90 - robot.arm.getArmPosition();

                if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                    arm_ik_target_x += gamepad2.left_stick_y * 12 * dt;
                }

                if (gamepad2.dpad_up) {
                    arm_ik_target_y += 6 * dt;
                }

                if (gamepad2.dpad_down) {
                    arm_ik_target_y -= 6 * dt;
                }

                if (gamepad2.dpad_left) {
                    claw_rot -= 90 * dt;
                }

                if (gamepad2.dpad_right) {
                    claw_rot += 90 * dt;
                }
            } else {
                if (gamepad2.dpad_up) {
                    arm_target += 20 * dt;
                }

                if (gamepad2.dpad_down) {
                    arm_target -= 20 * dt;
                }
            }

            robot.arm.setTarget(arm_target);
            robot.slides.setTarget(slides_target);

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
            //robot.slides.update(telemetry);

            // dont break claw while arm is retracted :(
            if (arm_target > -35) {
                robot.claw.rotate(claw_rot);
                robot.claw.bend(claw_bend);
            }

            //telemetry.addData("slides current", robot.slides.);
            //telemetry.addData("left stick", "%.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            //telemetry.addData("right stick", "%.2f, %.2f", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("robot state", arm_state);
            telemetry.update();

            prevGamepad1.copy(gamepad1);
            prevGamepad2.copy(gamepad2);
        }
    }
}
