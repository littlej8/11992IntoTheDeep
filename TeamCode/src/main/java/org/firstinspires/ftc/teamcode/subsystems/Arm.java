package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.opencv.core.Mat;

@Config
public class Arm implements Subsystem {
    DcMotorEx leftMotor, rightMotor;
    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;//0.4;
    public static double Kf = 0.4;//0.35;
    PIDController leftController = new PIDController(0, 0, 0),
                                rightController = new PIDController(0, 0, 0);
    double left = -50, right = -50;
    double degPerTick = 360.0 / 1497.325;
    public static double start_angle = -50;
    double target = -50;
    double profileTarget = -50, lastUpdate = System.currentTimeMillis();
    public static double DEG_PER_SEC = 90;
    public static double ANGLE_FINISH_DIST = 10;

    public static boolean USE_MOTION_PROFILE = true;

    Telemetry telemetry = null;

    public enum Position {
        RETRACTED,
        GRAB,
        LOW_HOOK,
        HIGH_HOOK,
        LOW_BASKET,
        HIGH_BASKET
    }
    public static double RETRACTED_POS = -50, GRAB_POS = -25, LOW_HOOK_POS = 45, HIGH_HOOK_POS = 75, LOW_BASKET_POS = 60, HIGH_BASKET_POS = 80;

    public Arm(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftMotor = hw.get(DcMotorEx.class, "armleft");
        rightMotor = hw.get(DcMotorEx.class, "armright");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Arm(HardwareMap hw, Telemetry tel) {
        this(hw);
        telemetry = tel;
    }

    private double enumToPos(Position pos) {
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

    public void setTarget(double deg) {
        target = deg;
    }

    public double getArmPosition() {
        return (left + right) / 2;
    }

    public Action goToAction(Position pos) {
        return goToAction(enumToPos(pos));
    }

    public Action goToAction(double deg) {
        return new Action() {
            boolean init = false;
            MotionProfile profile;
            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    setTarget(deg);
                    init = true;
                    profile = new MotionProfile(getArmPosition(), deg, 90, 90);
                }

                if (USE_MOTION_PROFILE) {
                    setTarget(profile.update());
                }
                update(telemetry);

                return !atPosition();
            }
        };
    }

    public void goToPosition(double deg, Telemetry tel) {
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

    public void goToPosition(double deg) {
        goToPosition(deg, telemetry);
    }

    public void goToPosition(Position pos) {
        goToPosition(enumToPos(pos));
    }

    public void goToPosition(Position pos, Telemetry tel) {
        goToPosition(enumToPos(pos), tel);
    }

    public boolean atPosition() {
        return Math.abs(target - left) < ANGLE_FINISH_DIST && Math.abs(target - right) < ANGLE_FINISH_DIST;
    }

    @Override
    public void update(Telemetry tel) {
        double angleDiff = Math.toRadians(target) - getArmPosition();

        if (!USE_MOTION_PROFILE) {
            double maxStep = DEG_PER_SEC * ((System.currentTimeMillis() - lastUpdate) / 1000);

            profileTarget += Math.min(maxStep, Math.abs(angleDiff)) * Math.signum(angleDiff);

            lastUpdate = System.currentTimeMillis();
        }

        leftController.setPID(Kp, Ki, Kd);
        rightController.setPID(Kp, Ki, Kd);

        double leftEnc = leftMotor.getCurrentPosition();
        double rightEnc = rightMotor.getCurrentPosition();

        double leftAngle = Math.toRadians(leftEnc * degPerTick) + Math.toRadians(start_angle);
        double rightAngle = Math.toRadians(rightEnc * degPerTick) + Math.toRadians(start_angle);
        double targetAngle = Math.toRadians(profileTarget);

        double leftPower = leftController.update(leftAngle, targetAngle) + (Math.cos(leftAngle) * Kf);
        double rightPower = rightController.update(rightAngle, targetAngle) + (Math.cos(rightAngle) * Kf);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (tel != null) {
            telemetry.addData("angle diff", Math.toDegrees(angleDiff));
            telemetry.addData("target", Math.toDegrees(targetAngle));
            telemetry.addData("left cur", leftEnc);
            telemetry.addData("right cur", rightEnc);
            telemetry.addData("left angle", Math.toDegrees(leftAngle));
            telemetry.addData("right angle", Math.toDegrees(rightAngle));
            telemetry.addData("left power", leftPower);
            telemetry.addData("right power", rightPower);
            telemetry.update();
        }

        left = leftAngle;
        right = rightAngle;
    }

    public void update() {
        update(telemetry);
    }
}
