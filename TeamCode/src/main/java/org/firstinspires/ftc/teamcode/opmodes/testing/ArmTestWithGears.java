package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@TeleOp(name="Arm Testing", group="Dashboard Tests")
public class ArmTestWithGears extends LinearOpMode {
    DcMotorEx motor;
    public static double Kp = 1.5;
    public static double Ki = 0.0;
    public static double Kd = 0;
    public static double Kf = 0.15;
    public static double Kl = 0.05;
    public static PIDController controller = new PIDController(0, 0, 0);
    public static double target = -40;
    public static double max_speed = 90;
    double setpoint = startAngle;
    public static double ticksToAngle = 5281.1 / 360.0;//(1497.325 * 1.75) / 360.0;
    public static double startAngle = -54;
    public static double max_power = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double max_adjust = max_speed * timer.seconds();
            double err = target - setpoint;
            setpoint += Math.min(Math.abs(err), max_adjust) * Math.signum(err);
            if (Math.abs(err) < 3) {
                setpoint = target;
            }
            timer.reset();

            controller.setPID(Kp, Ki, Kd);

            double enc = motor.getCurrentPosition();
            double angle = Math.toRadians(enc / ticksToAngle) + Math.toRadians(startAngle);
            double targetAngle = Math.toRadians(setpoint);
            double power = controller.update(angle, targetAngle) + (Math.cos(angle) * Kf) + (Math.signum(targetAngle - angle) * Kl);

            if (Math.abs(power) > max_power) {
                power = ((power < 0) ? -1 : 1) * max_power;
            }

            motor.setPower(power);

            telemetry.addData("target", Math.toDegrees(targetAngle));
            telemetry.addData("encoder", enc);
            telemetry.addData("angle", Math.toDegrees(angle));
            telemetry.addData("power", power);
            telemetry.addData("current", motor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}
