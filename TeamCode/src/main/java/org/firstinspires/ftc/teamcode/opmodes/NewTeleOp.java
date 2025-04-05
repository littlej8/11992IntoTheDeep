package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.NewArm;
import org.firstinspires.ftc.teamcode.subsystems.NewRobot;

/*
    yes
 */

@Config
@TeleOp(name="NewTeleOp", group="TeleOp")
public class NewTeleOp extends LinearOpMode {
    public enum ArmState {
        ARM_RETRACTED,
        ARM_FLOOR_PICKUP,
        ARM_FLOOR_GRABBING,
        ARM_WALL_PICKUP,
        ARM_HOOK,
        ARM_DROP,
        ARM_DROP_LOW
    }

    NewRobot robot;
    ArmState armState = ArmState.ARM_RETRACTED;
    boolean switchingStates = false;
    ElapsedTime dtTimer = new ElapsedTime();
    ElapsedTime switchTimer = new ElapsedTime();

    private void switchState(ArmState state) {
        armState = state;
        switchingStates = true;
        switchTimer.reset();
    }

    // prevent rapid switching from same button switch
    private boolean canSwitch() {
        return switchTimer.seconds() > 0.5;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new NewRobot(hardwareMap, 1.0, false);
        NewArm.max_speed = 90;

        boolean stayGripped = false;

        waitForStart();

        dtTimer.reset();

        while (opModeIsActive()) {
            double dt = dtTimer.seconds();
            dtTimer.reset();

            switch (armState) {
                case ARM_RETRACTED: {
                    if (switchingStates) {
                        robot.arm.setTarget(-40);
                        robot.slides.setTarget(0);
                        robot.claw.grip();
                        robot.claw.bend(0);
                        robot.claw.rotate(70);
                        switchingStates = false;
                    }

                    if (gamepad2.cross && canSwitch()) {
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    if (gamepad2.circle) {
                        switchState(ArmState.ARM_WALL_PICKUP);
                    }

                    if (gamepad2.triangle) {
                        switchState(ArmState.ARM_DROP);
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.right_trigger > 0.5) {
                        robot.claw.grip();
                    }

                    break;
                }

                case ARM_FLOOR_PICKUP: {
                    if (switchingStates) {
                        robot.claw.bend(135);
                        switchingStates = false;

                        if (!stayGripped) {
                            robot.arm.setTarget(-35);
                            robot.claw.rotate(70);
                            robot.claw.drop();
                            robot.slides.setTarget(0);
                        } else {
                            robot.arm.setTarget(robot.arm.getTarget() + 10);
                            robot.slides.setTarget(robot.slides.getTarget()-3);
                        }

                        stayGripped = false;
                    }

                    if (gamepad2.cross && canSwitch()) {
                        switchState(ArmState.ARM_RETRACTED);
                    }

                    if (gamepad2.circle) {
                        switchState(ArmState.ARM_HOOK);
                    }

                    if (gamepad2.triangle) {
                        switchState(ArmState.ARM_DROP);
                    }

                    if (gamepad2.right_trigger > 0.5 && canSwitch()) {
                        switchState(ArmState.ARM_FLOOR_GRABBING);
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.left_bumper) {
                        robot.claw.rotateBy(90 * dt);
                    }

                    if (gamepad2.right_bumper) {
                        robot.claw.rotateBy(-90 * dt);
                    }

                    if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                        double newSlides = robot.slides.getTarget() + (-gamepad2.left_stick_y * (10 * dt));
                        if (newSlides <= 18 && newSlides > 0) {
                            robot.arm.setTarget(robot.arm.getTarget() + (-gamepad2.left_stick_y) * (10 * dt));
                            robot.slides.setTarget(newSlides);
                        }
                        if (newSlides > 12) {
                            robot.claw.bend(145);
                        } else {
                            robot.claw.bend(135);
                        }
                    }

                    break;
                }

                case ARM_FLOOR_GRABBING: {
                    if (switchingStates) {
                        robot.arm.setTarget(robot.arm.getTarget()-10);
                        robot.slides.setTarget(robot.slides.getTarget()+3);
                        robot.claw.bend(140);
                        robot.claw.drop();
                        switchingStates = false;
                    }

                    if ((robot.arm.moveFinished() && robot.claw.atPosition()) || switchTimer.seconds() > 0.5) {
                        robot.claw.grip();
                    }

                    if (switchTimer.seconds() > 1) {
                        stayGripped = true;
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    break;
                }

                case ARM_WALL_PICKUP: {
                    if (switchingStates) {
                        robot.arm.setTarget(-25);
                        robot.slides.setTarget(0);
                        robot.claw.bend(60);
                        robot.claw.rotate(70);
                        robot.claw.drop();
                        switchingStates = false;
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.right_trigger > 0.5) {
                        robot.claw.grip();
                    }

                    if (gamepad2.circle && canSwitch()) {
                        switchState(ArmState.ARM_HOOK);
                    }

                    if (gamepad2.cross) {
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    if (gamepad2.triangle) {
                        switchState(ArmState.ARM_DROP);
                    }

                    break;
                }

                case ARM_HOOK: {
                    if (switchingStates) {
                        robot.arm.setTarget(9);
                        robot.slides.setTarget(0);
                        robot.claw.bend(20);
                        robot.claw.rotate(70);
                        switchingStates = false;
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.right_trigger > 0.5) {
                        robot.claw.grip();
                    }

                    if (gamepad2.circle && canSwitch()) {
                        switchState(ArmState.ARM_WALL_PICKUP);
                    }

                    if (gamepad2.cross) {
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    if (gamepad2.triangle) {
                        switchState(ArmState.ARM_DROP);
                    }

                    break;
                }

                case ARM_DROP: {
                    if (switchingStates) {
                        robot.slides.setTarget(0);
                        robot.claw.bend(30);
                        robot.claw.rotate(80);
                        switchingStates = false;
                    }

                    if (switchTimer.seconds() > 1) {
                        robot.arm.setTarget(80);
                    }

                    if (switchTimer.seconds() > 2 && robot.arm.moveFinished()) {
                        robot.slides.setTarget(29);
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.right_trigger > 0.5) {
                        robot.claw.grip();
                    }

                    if (gamepad2.cross) {
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    if (gamepad2.circle) {
                        switchState(ArmState.ARM_HOOK);
                    }

                    if (gamepad2.right_bumper && switchTimer.seconds() > 0.5) {
                        switchState(ArmState.ARM_DROP_LOW);
                    }

                    break;
                }

                case ARM_DROP_LOW: {
                    if (switchingStates) {
                        robot.slides.setTarget(0);
                        robot.claw.bend(30);
                        robot.claw.rotate(80);
                        switchingStates = false;
                    }

                    if (switchTimer.seconds() > 1) {
                        robot.arm.setTarget(80);
                    }

                    if (gamepad2.left_trigger > 0.5) {
                        robot.claw.drop();
                    }

                    if (gamepad2.right_trigger > 0.5) {
                        robot.claw.grip();
                    }

                    if (gamepad2.cross) {
                        switchState(ArmState.ARM_FLOOR_PICKUP);
                    }

                    if (gamepad2.circle) {
                        switchState(ArmState.ARM_HOOK);
                    }

                    if (gamepad2.right_bumper && switchTimer.seconds() > 0.5) {
                        switchState(ArmState.ARM_DROP);
                    }

                    break;
                }

                default: {
                    switchState(ArmState.ARM_RETRACTED);
                    break;
                }
            }

            if (gamepad2.square) {
                switchState(ArmState.ARM_RETRACTED);
            }

            if (gamepad2.dpad_up) {
                NewArm.startAngle -= 30 * dt;
            }

            if (gamepad2.dpad_down) {
                NewArm.startAngle += 30 * dt;
            }

            robot.arm.update(telemetry);
            robot.slides.update(telemetry);

            robot.dt.driveTeleOp(gamepad1);

            if (gamepad1.dpad_up) {
                robot.lift.setPowers(1.0);
            } else if (gamepad1.dpad_down) {
                robot.lift.setPowers(-1.0);
            } else {
                robot.lift.setPowers(0.0);
            }

            telemetry.update();
        }
    }
}
