package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTester extends LinearOpMode {
    public static double MOTOR_POWER = 0.2;
    public static String MOTOR_NAME = "Arm";
    public static double degPerTick = (360 / 1497.325 / 2);
    public static double other = 180;
    public static double targetDeg = other;
    boolean left = true;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double motorAngle = motor.getCurrentPosition() * degPerTick;
            double error = (targetDeg - motorAngle);
            double power = error * 0.15;
            if (Math.abs(power) > MOTOR_POWER) {
                power = (power / Math.abs(power)) * MOTOR_POWER;
            }
            power += Math.cos(Math.toRadians(motorAngle)) * 0.0;//75;
            motor.setPower(power);
            telemetry.addData("angle", motorAngle);
            telemetry.addData("target", targetDeg);
            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.update();

            if (Math.abs(error) < 1 && left) {
                targetDeg = 0;
                left = false;
            } else if (Math.abs(error) < 1 && !left) {
                targetDeg = other;
                left = true;
            }
        }
    }
}
