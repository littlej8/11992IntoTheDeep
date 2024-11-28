package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements Subsystem {
    Servo rotLeft, rotRight;
    Servo wrist;
    Servo grip;

    double powerPerDeg = 1.0 / 180.0;

    public Claw(HardwareMap hwMap) {
        rotLeft = hwMap.get(Servo.class, "rotLeft");
        rotRight = hwMap.get(Servo.class, "rotRight");
        wrist = hwMap.get(Servo.class, "wrist");
        grip = hwMap.get(Servo.class, "grip");

        rotLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void rotate(int deg) {
        rotLeft.setPosition(deg * powerPerDeg);
        rotRight.setPosition(deg * powerPerDeg);
    }

    public void rotateWrist(int deg) {
        wrist.setPosition(deg * powerPerDeg);
    }

    public void grip() {
        grip.setPosition(1);
    }

    public void drop() {
        grip.setPosition(-1);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
