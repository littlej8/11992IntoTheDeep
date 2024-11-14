package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.shuttle.HardwareTaskScope;

public class Robot {
    Drivetrain dt;
    Arm arm;
    Slides slides;

    public Robot(HardwareMap hw, Telemetry telemetry) {
        dt = new Drivetrain(hw, telemetry);
        arm = new Arm(hw, telemetry);
        slides = new Slides(hw, telemetry);
    }

    public Robot(HardwareMap hw) {
        dt = new Drivetrain(hw);
        arm = new Arm(hw);
        slides = new Slides(hw);
    }

    public void move(double x, double y, double h) throws InterruptedException {
        try (HardwareTaskScope<InterruptedException> scope = HardwareTaskScope.open()) {
            scope.fork(() -> dt.moveTo(x, y, h));
            scope.fork(() -> {
                while (!dt.moveFinished()) arm.update();
            });
            scope.fork(() -> {
                while (!dt.moveFinished()) slides.update();
            });

            scope.join();
        }
    }

    public void maintain(double millis) throws InterruptedException {
        try (HardwareTaskScope<InterruptedException> scope = HardwareTaskScope.open()) {
            scope.fork(() -> dt.maintainPosition(millis));
            scope.fork(() -> {
                while (!dt.moveFinished()) arm.update();
            });
            scope.fork(() -> {
                while (!dt.moveFinished()) slides.update();
            });

            scope.join();
        }
    }
}
