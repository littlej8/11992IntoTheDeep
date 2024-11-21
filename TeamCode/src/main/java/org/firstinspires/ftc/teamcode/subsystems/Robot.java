package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;
import org.firstinspires.ftc.teamcode.util.shuttle.HardwareTaskScope;

public class Robot {
    ActionScheduler scheduler = new ActionScheduler();
    Telemetry telemetry;
    Drivetrain dt;
    Arm arm;
    Slides slides;

    public Robot(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        dt = new Drivetrain(hw, telemetry);
        arm = new Arm(hw, telemetry);
        slides = new Slides(hw, telemetry);
    }

    public Robot(HardwareMap hw) {
        this(hw, new NullTelemetry());
    }

    public Robot(HardwareMap hw, Telemetry telemetry, Pose2d startPose, boolean onlyDt) {
        this.telemetry = telemetry;
        if (onlyDt) {
            dt = new Drivetrain(hw, telemetry, startPose);
        } else {
            arm = new Arm(hw, telemetry);
            slides = new Slides(hw, telemetry);
        }
    }

    public void schedule(Action... actions) {
        scheduler.schedule(actions);
    }

    public void update() {
        scheduler.runNextAction(telemetry);
    }

    public Action moveAction(double x, double y, double h) {
        return new Action() {
            boolean init = false;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    dt.setTargetPose(new Pose2d(x, y, h));
                    init = true;
                }

                if (dt.moveFinished())
                    return false;

                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                return true;
            }
        };
    }

    public Action highHookAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action hookSpecimenAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action grabSpecimenAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
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
