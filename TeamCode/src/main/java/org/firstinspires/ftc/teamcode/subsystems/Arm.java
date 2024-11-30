package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Arm implements Subsystem {
    DcMotorEx armMotor;
    public static double Kp = 1.5;
    public static double Ki = 0.0;
    public static double Kd = 0.4;
    public static double Kf = 0.35;
    public static PIDController controller = new PIDController(0, 0, 0);
    public static double current = 0, target = 0;
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
    public static double RETRACTED_POS = -50, GRAB_POS = -25, LOW_HOOK_POS = 45, HIGH_HOOK_POS = 75, LOW_BASKET_POS = 60, HIGH_BASKET_POS = 80;

    public Arm(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        armMotor = hw.get(DcMotorEx.class, "Arm");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        return current;
    }

    public Action goToAction(Position pos) {
        return goToAction(enumToPos(pos));
    }

    public Action goToAction(double deg) {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    setTarget(deg);
                    init = true;
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
        return Math.abs(target - current) < ANGLE_FINISH_DIST;
    }

    @Override
    public void update(Telemetry tel) {
        controller.setPID(Kp, Ki, Kd);

        double armEnc = armMotor.getCurrentPosition();

        double armAngle = Math.toRadians((armEnc + startTicks) * degPerTick);
        double targetAngle = Math.toRadians(target);

        double armPower = controller.update(armAngle, targetAngle) + (Math.cos(armAngle) * Kf);;

        armMotor.setPower(armPower);

        if (tel != null) {
            tel.addData("target", Math.toDegrees(targetAngle));
            tel.addData("arm cur", armEnc);
            tel.addData("arm angle", Math.toDegrees(armAngle));
            tel.addData("arm power", armPower);
        }

        current = armAngle;
    }

    public void update() {
        update(telemetry);
    }
}
