package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class NewArm implements Subsystem {
    DcMotorEx motor;
    public static double Kp = 1.5;
    public static double Ki = 0.0;
    public static double Kd = 0;
    public static double Kf = 0.15;
    public static double Kl = 0.05;
    public static PIDController controller = new PIDController(0, 0, 0);
    public static double target = -54;
    public static double max_speed = 135;
    double setpoint = startAngle;
    public static double ticksToAngle = 5281.1 / 360.0;//(1497.325 * 1.75) / 360.0;
    public static double startAngle = -54;
    public static double max_power = 1;

    private double currentAngle = startAngle;
    private final ElapsedTime timer;

    public NewArm(HardwareMap hw, boolean teleop) {
        motor = hw.get(DcMotorEx.class, "arm");
        if (!teleop)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
    }

    public NewArm(HardwareMap hw) {
        this(hw, false);
    }

    public double getTarget() {
        return target;
    }
    public void setTarget(double deg) {
        target = deg;
    }

    // radians
    public double getAngle() {
        return currentAngle;
    }
    public boolean moveFinished() {
        return Math.abs(getTarget() - Math.toDegrees(getAngle())) < 5;
    }

    // ram into robot for a second so backlash is always the same
    public void eliminateBacklash() {
        ElapsedTime timer = new ElapsedTime();
        motor.setPower(-0.1);
        while (timer.seconds() < 3 && Thread.currentThread().isAlive()) {
            // do nothing
        }
        motor.setPower(0);
    }

    public void kill() {
        motor.setPower(0);
    }

    @Override
    public void update(Telemetry telemetry) {
        if (timer.seconds() > 0.1) {
            timer.reset();
        }

        double max_adjust = max_speed * timer.seconds();
        double err = target - setpoint;
        setpoint += Math.min(Math.abs(err), max_adjust) * Math.signum(err);
        if (Math.abs(err) < 3) {
            setpoint = target;
        }
        timer.reset();

        controller.setPID(Kp, Ki, Kd);

        double enc = motor.getCurrentPosition();
        currentAngle = Math.toRadians(enc / ticksToAngle) + Math.toRadians(startAngle);
        double targetAngle = Math.toRadians(setpoint);
        double power = controller.update(currentAngle, targetAngle) + (Math.cos(currentAngle) * Kf) + (Math.signum(targetAngle - currentAngle) * Kl);

        if (Math.abs(power) > max_power) {
            power = ((power < 0) ? -1 : 1) * max_power;
        }

        motor.setPower(power);

        telemetry.addData("arm target", Math.toDegrees(targetAngle));
        telemetry.addData("arm angle", Math.toDegrees(currentAngle));
        telemetry.addData("arm power", power);
        telemetry.addData("arm milliamps", motor.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
