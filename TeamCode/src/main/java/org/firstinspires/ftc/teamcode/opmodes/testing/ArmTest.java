package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
@TeleOp
@Disabled
public class ArmTest extends LinearOpMode {
    DcMotorEx leftMotor, rightMotor;
    Slides slides;
    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.4;
    public static PIDController leftController = new PIDController(0, 0, 0);
    public static PIDController rightController = new PIDController(0, 0, 0);
    public static double target = 0;
    public static double slidesTarget = 0;
    public static double ticksToAngle = (1497.325) / 360.0;//(1497.325 * 1.75) / 360.0;
    public static double startAngle = -50.0;
    public static double max_speed = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap, telemetry);

        leftMotor = hardwareMap.get(DcMotorEx.class, "armleft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "armright");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while (opModeIsActive()) {
            leftController.setPID(Kp, Ki, Kd);
            rightController.setPID(Kp, Ki, Kd);

            double leftEnc = leftMotor.getCurrentPosition();
            double leftAngle = Math.toRadians(leftEnc / ticksToAngle) + Math.toRadians(startAngle);
            double rightEnc = rightMotor.getCurrentPosition();
            double rightAngle = Math.toRadians(rightEnc / ticksToAngle) + Math.toRadians(startAngle);
            double targetAngle = Math.toRadians(target);
            double leftPower = rightController.update(leftAngle, targetAngle) + (Math.cos(leftAngle) * Kf);
            double rightPower = rightController.update(rightAngle, targetAngle) + (Math.cos(rightAngle) * Kf);

            if (Math.abs(rightPower) > max_speed) {
                rightPower = ((rightPower < 0) ? -1 : 1) * max_speed;
            }

            if (Math.abs(leftPower) > max_speed) {
                leftPower = ((leftPower < 0) ? -1 : 1) * max_speed;
            }

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            slides.setTarget(slidesTarget);
            slides.update(telemetry);

            telemetry.addData("target", Math.toDegrees(targetAngle));
            telemetry.addData("left cur", leftEnc);
            telemetry.addData("right cur", rightEnc);
            telemetry.addData("left angle", Math.toDegrees(leftAngle));
            telemetry.addData("right angle", Math.toDegrees(rightAngle));
            telemetry.addData("left power", leftPower);
            telemetry.addData("right power", rightPower);
            telemetry.update();
        }
    }
}
