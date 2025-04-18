package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Config
public class Arm implements Subsystem {
    DcMotorEx motor;
    public static double Kp = 5.0;
    public static double Ki = 0;
    public static double Kd = 0.01;//0.4;
    public static double Kf = 0.2;//0.35;
    PIDController controller = new PIDController(0, 0, 0);
    public static double currentAngle = -40;
    double degPerTick = 360.0 / (145.1);
    public static double start_angle = -40;
    double target = -40;
    double profileTarget = -40, lastUpdate = System.currentTimeMillis();
    public static double DEG_PER_SEC = 180;
    public static double ANGLE_FINISH_DIST = 10;
    public static double MAX_POWER = 0.7;

    public static boolean USE_MOTION_PROFILE = false;
    boolean finishedMove = true;

    Telemetry telemetry = null;

    public enum Position {
        RETRACTED,
        GRAB,
        LOW_HOOK,
        HIGH_HOOK,
        LOW_BASKET,
        HIGH_BASKET
    }
    public static double RETRACTED_POS = -40, GRAB_POS = -5, LOW_HOOK_POS = 0, HIGH_HOOK_POS = 60, LOW_BASKET_POS = 60, HIGH_BASKET_POS = 80;
    double maxVoltage;
    public static boolean ADJUST_UP_5_IF_VOLTAGE_LOW = false;
    public static boolean LIMIT_DOWN_POWER_ABOVE_45 = true;

    public Arm(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        motor = hw.get(DcMotorEx.class, "arm");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        maxVoltage = hw.voltageSensor.iterator().next().getVoltage();

        start_angle = PoseSingleton.getInstance().getArmAngle();
    }

    public void set_start_angle(double angle) {
        start_angle = angle;
    }

    public void decreaseStartAngle(double amt) {
        start_angle -= amt;
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
        finishedMove = false;
        /*if (maxVoltage < 13.1 && ADJUST_UP_5_IF_VOLTAGE_LOW) {
            target = deg + 6;
        } else if (maxVoltage < 13.5 && ADJUST_UP_5_IF_VOLTAGE_LOW) {
            target = deg + 5;
        } else {*/
            target = deg;
        //}
    }

    public double getArmPosition() {
        return currentAngle;
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
                    profile = new MotionProfile(getArmPosition(), deg, 90, 45);
                }

                if (USE_MOTION_PROFILE) {
                    profileTarget = profile.update();
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
        return Math.abs(target - Math.toDegrees(getArmPosition())) < ANGLE_FINISH_DIST;
    }

    @Override
    public void update(Telemetry tel) {
        double angleDiff = target - Math.toDegrees(getArmPosition());

        if (!USE_MOTION_PROFILE) {
            double seconds = (System.currentTimeMillis() - lastUpdate) / 1000;
            double maxStep = DEG_PER_SEC * seconds;
            double err = target - profileTarget;
            profileTarget += Math.min(Math.abs(err), maxStep) * Math.signum(err);
            if (Math.abs(err) < 3) {
                profileTarget = target;
            }

            lastUpdate = System.currentTimeMillis();
        }

        controller.setPID(Kp, Ki, Kd);

        double enc = motor.getCurrentPosition();

        double angle = Math.toRadians(enc * degPerTick) + Math.toRadians(start_angle);
        double targetAngle = Math.toRadians(profileTarget);

        double power = controller.update(angle, targetAngle) + (Math.cos(angle) * Kf);

        if (Math.abs(power) > MAX_POWER) {
            power = MAX_POWER * Math.signum(power);
        }


        if (power < 0.005 && getArmPosition() < Math.toRadians(45)) {
            power = 0.005;
        } else if (power < -0.2 && LIMIT_DOWN_POWER_ABOVE_45) {
            power = -0.2;
        }

        motor.setPower(power);

        if (tel != null) {
            telemetry.addData("angle diff", angleDiff);
            telemetry.addData("target", Math.toDegrees(targetAngle));
            telemetry.addData("angle", Math.toDegrees(angle));
            telemetry.addData("power", power);
        }

        currentAngle = angle;
    }

    public void update() {
        update(telemetry);
    }
}
