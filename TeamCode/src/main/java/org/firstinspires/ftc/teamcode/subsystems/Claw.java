package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

public class Claw implements Subsystem {
    Servo wristRot;
    Servo wristBend;
    Servo grip;

    double powerPerDegRot = 1.0 / 180.0;
    double powerPerDegBend = 1.0 / 270.0;

    public static double millisPerDeg = 5;
    public static double gripMillis = 250;

    public Claw(HardwareMap hwMap) {
        wristRot = hwMap.get(Servo.class, "wrist_rot");
        wristBend = hwMap.get(Servo.class, "wrist_bend");
        grip = hwMap.get(Servo.class, "grip");
    }

    public void rotate(double deg) {
        wristRot.setPosition(deg * powerPerDegRot);
    }

    public void bend(double deg) {
        wristBend.setPosition(deg * powerPerDegBend);
    }

    public void drop() {
        grip.setPosition(1);
    }

    public void grip() {
        grip.setPosition(0);
    }

    public Action rotateAction(double deg) {
        return new ActionSequence(
                telemetry -> {
                    rotate(deg);
                    return false;
                },
                new WaitAction(Math.abs(deg - wristRot.getPosition()) * millisPerDeg)
        );
    }

    public Action gripAction() {
        return new ActionSequence(
                telemetry -> {
                    drop();
                    return false;
                },
                new WaitAction(gripMillis)
        );
    }

    public Action dropAction() {
        return new ActionSequence(
                telemetry -> {
                    grip();
                    return false;
                },
                new WaitAction(gripMillis)
        );
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
