package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
@TeleOp
public class ArmTest extends LinearOpMode {
    DcMotorEx rightMotor;//leftMotor, rightMotor;
    public static double Kp = 1.5;
    public static double Ki = 0.0;
    public static double Kd = 0.4;
    public static double Kf = 0.35;
    public static PIDController leftController = new PIDController(0, 0, 0);
    public static PIDController rightController = new PIDController(0, 0, 0);
    public static int target = 0;
    public static double ticksToAngle = 145.1 / 360.0;
    public static double startTicks = -2;
    @Override
    public void runOpMode() throws InterruptedException {
        //leftMotor = hardwareMap.get(DcMotorEx.class, "armleft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "armright");
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while (opModeIsActive()) {
            rightController.setKp(Kp);
            rightController.setKi(Ki);
            rightController.setKd(Kd);

            //double leftEnc = leftMotor.getCurrentPosition();
            double rightEnc = rightMotor.getCurrentPosition();
            double rightAngle = Math.toRadians((rightEnc + startTicks) / ticksToAngle);
            double targetAngle = Math.toRadians(target);
            //double leftPower = leftController.update(leftEnc, target);
            double rightPower = rightController.update(rightAngle, targetAngle) + (Math.cos(rightAngle) * Kf);

            //leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            telemetry.addData("target", Math.toDegrees(targetAngle));
            //telemetry.addData("left cur", leftEnc);
            telemetry.addData("right cur", rightEnc);
            telemetry.addData("right angle", Math.toDegrees(rightAngle));
            //telemetry.addData("left power", leftPower);
            telemetry.addData("right power", rightPower);
            telemetry.update();
        }
    }
}
