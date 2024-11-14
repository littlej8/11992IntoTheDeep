package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static double leftCurrent = 0, rightCurrent = 0, target = 0;
    public static double degPerTick = 360.0 / 384.5;
    public static double startTicks = -5;
    public static double ANGLE_FINISH_DIST = 5;

    Telemetry telemetry = null;

    public enum Position {
        RETRACTED,
        GRAB,
        LOW_HOOK,
        HIGH_HOOK,
        LOW_BASKET,
        HIGH_BASKET
    }
    public static int RETRACTED_POS = -4, GRAB_POS = 2, LOW_HOOK_POS = 45, HIGH_HOOK_POS = 75, LOW_BASKET_POS = 60, HIGH_BASKET_POS = 80;

    public Arm(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftMotor = hw.get(DcMotorEx.class, "armleft");
        rightMotor = hw.get(DcMotorEx.class, "armright");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Arm(HardwareMap hw, Telemetry tel) {
        this(hw);
        telemetry = tel;
    }

    private int enumToPos(Position pos) {
        switch (pos) {
            case GRAB: return GRAB_POS;
            case LOW_HOOK: return LOW_HOOK_POS;
            case HIGH_HOOK: return HIGH_HOOK_POS;
            case LOW_BASKET: return LOW_BASKET_POS;
            case HIGH_BASKET: return HIGH_BASKET_POS;
            default: return RETRACTED_POS;
        }
    }

    public void setTarget(Position pos) {
        setTarget(enumToPos(pos));
    }

    public void setTarget(int deg) {
        target = deg;
    }

    public void goToPosition(int deg, Telemetry tel) {
        target = deg;
        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            if (tel != null) {
                update(tel);
                tel.update();
            } else {
                update();
            }
            running = !atPosition();
        }
    }

    public void goToPosition(int deg) {
        goToPosition(deg, telemetry);
    }

    public void goToPosition(Position pos) {
        goToPosition(enumToPos(pos));
    }

    public void goToPosition(Position pos, Telemetry tel) {
        goToPosition(enumToPos(pos), tel);
    }

    public boolean atPosition() {
        return Math.abs(target - ((leftCurrent + rightCurrent) / 2)) < ANGLE_FINISH_DIST;
    }

    public void update(Telemetry tel) {
        leftController.setPID(Kp, Ki, Kd);
        rightController.setPID(Kp, Ki, Kd);

        double leftEnc = leftMotor.getCurrentPosition();
        double rightEnc = rightMotor.getCurrentPosition();

        double leftAngle = Math.toRadians((leftEnc + startTicks) * degPerTick);
        double rightAngle = Math.toRadians((rightEnc + startTicks) * degPerTick);
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

        leftCurrent = leftAngle;
        rightCurrent = rightAngle;
    }

    public void update() {
        update(telemetry);
    }
}
