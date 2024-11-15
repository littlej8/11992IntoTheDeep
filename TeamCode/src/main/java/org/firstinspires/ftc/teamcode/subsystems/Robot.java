package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public void moveAndRaiseArmToHighHook(double x, double y, double h) throws InterruptedException {
        arm.setTarget(Arm.Position.HIGH_HOOK);
        slides.setTarget(Slides.Position.HIGH_HOOK);
        move(x, y, h);
    }

    public void moveAndRaiseArmToLowHook(double x, double y, double h) throws InterruptedException {
        arm.setTarget(Arm.Position.LOW_HOOK);
        slides.setTarget(Slides.Position.LOW_HOOK);
        move(x, y, h);
    }

    public void moveAndRaiseArmToHighBasket(double x, double y, double h) throws InterruptedException {
        arm.setTarget(Arm.Position.HIGH_BASKET);
        slides.setTarget(Slides.Position.HIGH_BASKET);
        move(x, y, h);
    }

    public void moveAndRaiseArmToLowBasket(double x, double y, double h) throws InterruptedException {
        arm.setTarget(Arm.Position.LOW_BASKET);
        slides.setTarget(Slides.Position.LOW_BASKET);
        move(x, y, h);
    }

    public void hookSpecimen() throws InterruptedException {
        try (HardwareTaskScope<InterruptedException> scope = HardwareTaskScope.open()) {
            scope.fork(() -> {
                while (!slides.atPosition()) {
                    dt.updatePose();
                    dt.updateMovement();
                }
            });
            scope.fork(() -> {
                while (!slides.atPosition()) arm.update();
            });
            scope.fork(slides::hook);

            scope.join();
        }
    }

    public void maintain(double millis) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        try (HardwareTaskScope<InterruptedException> scope = HardwareTaskScope.open()) {
            scope.fork(() -> dt.maintainPosition(millis));
            scope.fork(() -> {
                while (timer.milliseconds() < millis) arm.update();
            });
            scope.fork(() -> {
                while (timer.milliseconds() < millis) slides.update();
            });

            scope.join();
        }
    }
}
