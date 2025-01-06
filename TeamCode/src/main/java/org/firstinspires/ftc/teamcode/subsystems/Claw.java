package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

public class Claw implements Subsystem {
    Servo wristRot;
    Servo wristInc;
    Servo grip;

    double powerPerDeg = 1.0 / 180.0;

    public static double millisPerDeg = 5;
    public static double gripMillis = 250;

    public Claw(HardwareMap hwMap) {
        wristRot = hwMap.get(Servo.class, "wristRot");
        wristInc = hwMap.get(Servo.class, "wristInc");
        grip = hwMap.get(Servo.class, "grip");
    }

    public void rotate(int deg) {
        wristRot.setPosition(deg * powerPerDeg);
    }

    public void inc(int deg) {
        wristInc.setPosition(deg * powerPerDeg);
    }

    public void grip() {
        grip.setPosition(1);
    }

    public void drop() {
        grip.setPosition(0);
    }

    public Action rotateAction(int deg) {
        rotate(deg);
        return new WaitAction(deg * millisPerDeg);
    }

    public Action gripAction() {
        grip();
        return new WaitAction(gripMillis);
    }

    public Action dropAction() {
        drop();
        return new WaitAction(gripMillis);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
