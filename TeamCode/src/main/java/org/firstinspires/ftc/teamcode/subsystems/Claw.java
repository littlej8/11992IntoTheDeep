package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

public class Claw implements Subsystem {
    //Servo rotLeft, rotRight;
    Servo wrist;
    Servo grip;

    double powerPerDeg = 1.0 / 180.0;

    public static double millisPerDeg = 5;
    public static double gripMillis = 250;

    public Claw(HardwareMap hwMap) {
        //rotLeft = hwMap.get(Servo.class, "rotLeft");
        //rotRight = hwMap.get(Servo.class, "rotRight");
        wrist = hwMap.get(Servo.class, "wrist");
        grip = hwMap.get(Servo.class, "grip");

        //rotLeft.setDirection(Servo.Direction.REVERSE);
    }

    /*public void rotate(int deg) {
        rotLeft.setPosition(deg * powerPerDeg);
        rotRight.setPosition(deg * powerPerDeg);
    }*/

    public void rotate(int deg) {
        wrist.setPosition(deg * powerPerDeg);
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
