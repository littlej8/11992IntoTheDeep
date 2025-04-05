package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Slides implements Subsystem {
    DcMotorEx slideMotor;
    public static double Kp = 0.5;
    public static double Ki = 0.0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static PIDController slideController = new PIDController(0, 0, 0);
    double current = 0, target = 0;
    public static double FINISH_DIST = 1.0;

    public static double pulleyDiameter = 2.0;
    public static double inPerTick = (pulleyDiameter * Math.PI) / 384.5;

    Telemetry telemetry;

    public enum Position {
        RETRACTED,
        GRAB,
        LOW_HOOK,
        HIGH_HOOK,
        LOW_BASKET,
        HIGH_BASKET
    }
    public static double RETRACTED_POS = 0, GRAB_POS = 2, LOW_HOOK_POS = 10, HIGH_HOOK_POS = 20, LOW_BASKET_POS = 15, HIGH_BASKET_POS = 30;

    public Slides(HardwareMap hw) {
        for (LynxModule module :hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        slideMotor = hw.get(DcMotorEx.class, "slides");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Slides(HardwareMap hw, Telemetry tel) {
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

    public void setTarget(double pos) {
        target = Math.max(0, Math.min(29, pos));
    }

    public double getTarget() {
        return target;
    }

    public Action goToAction(Position pos) {
        return goToAction(enumToPos(pos));
    }

    public Action goToAction(double pos) {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    setTarget(pos);
                    init = true;
                }

                update(telemetry);

                if (atPosition()) {
                    slideMotor.setPower(0);
                }

                return !atPosition();
            }
        };
    }

    public void goToPosition(double pos, Telemetry tel) {
        target = pos;
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

    public void goToPosition(double pos) {
        goToPosition(pos, telemetry);
    }

    public void goToPosition(Position pos) {
        goToPosition(enumToPos(pos));
    }

    public void goToPosition(Position pos, Telemetry tel) {
        goToPosition(enumToPos(pos), tel);
    }

    public boolean atPosition() {
        return Math.abs(target - current) < FINISH_DIST;
    }

    public void hook() {
        goToPosition(target - 2);
    }

    public void resetPosition() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update(Telemetry tel) {
        slideController.setPID(Kp, Ki, Kd);

        double slideEnc = slideMotor.getCurrentPosition();

        double slideHeight = slideEnc * inPerTick;

        double slidePower = slideController.update(slideHeight, target) + Kf * Math.signum(target - slideHeight);

        slideMotor.setPower(slidePower);

        if (tel != null) {
            tel.addData("target", target);
            tel.addData("slide cur", slideEnc);
            tel.addData("slide height", slideHeight);
            tel.addData("slide power", slidePower);
        }

        current = slideHeight;
    }

    public void update() {
        update(null);
    }
}
