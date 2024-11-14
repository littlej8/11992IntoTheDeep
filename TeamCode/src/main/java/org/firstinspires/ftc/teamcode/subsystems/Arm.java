package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Arm {
    DcMotorEx leftMotor, rightMotor;
    public static double Kp = 1.5;
    public static double Ki = 0.0;
    public static double Kd = 0.4;
    public static double Kf = 0.35;
    public static PIDController leftController = new PIDController(0, 0, 0);
    public static PIDController rightController = new PIDController(0, 0, 0);
    public static int target = 0;
    public static double ticksToAngle = 384.5 / 360.0;
    public static double startTicks = -2;

    public Arm(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftMotor = hw.get(DcMotorEx.class, "armleft");
        rightMotor = hw.get(DcMotorEx.class, "armright");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(int deg) {
        target = deg;
    }

    public void update(Telemetry tel) {
        leftController.setPID(Kp, Ki, Kd);
        rightController.setPID(Kp, Ki, Kd);

        double leftEnc = leftMotor.getCurrentPosition();
        double rightEnc = rightMotor.getCurrentPosition();

        double leftAngle = Math.toRadians((leftEnc + startTicks) / ticksToAngle);
        double rightAngle = Math.toRadians((rightEnc + startTicks) / ticksToAngle);
        double targetAngle = Math.toRadians(target);

        double leftPower = leftController.update(leftAngle, targetAngle) + (Math.cos(leftAngle) * Kf);;
        double rightPower = rightController.update(rightAngle, targetAngle) + (Math.cos(rightAngle) * Kf);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (tel != null) {
            tel.addData("target", Math.toDegrees(targetAngle));
            tel.addData("left cur", leftEnc);
            tel.addData("right cur", rightEnc);
            tel.addData("left angle", Math.toDegrees(leftAngle));
            tel.addData("right angle", Math.toDegrees(rightAngle));
            tel.addData("left power", leftPower);
            tel.addData("right power", rightPower);
        }
    }

    public void update() {
        update(null);
    }
}
