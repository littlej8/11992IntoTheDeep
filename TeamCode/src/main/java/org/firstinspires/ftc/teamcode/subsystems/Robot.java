package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;

public class Robot {
    ActionScheduler scheduler = new ActionScheduler();
    Telemetry telemetry;
    public Drivetrain dt;
    public Arm arm;
    public Slides slides;

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

    public boolean actionsDone() {
        return scheduler.actionsDone();
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
                    dt.setTargetPose(new Pose2d(x, y, Math.toRadians(h)));
                    init = true;
                }

                if (dt.moveFinished()) {
                    dt.killPowers();
                    return false;
                }

                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                return true;
            }
        };
    }

    public Action moveAction(double x, double y, double h, double speed) {
        return new Action() {
            boolean init = false;
            final double prevPower = Drivetrain.MAX_WHEEL_POWER;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    Drivetrain.MAX_WHEEL_POWER = speed;
                    dt.setTargetPose(new Pose2d(x, y, Math.toRadians(h)));
                    init = true;
                }

                if (dt.moveFinished()) {
                    Drivetrain.MAX_WHEEL_POWER = prevPower;
                    dt.killPowers();
                    return false;
                }

                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                return true;
            }
        };
    }

    public Action moveAndAction(double x, double y, double h, Action action) {
        return new UntilAction(moveAction(x, y, h), action);
    }

    public Action moveAndAction(double x, double y, double h, double speed, Action action) {
        return new UntilAction(moveAction(x, y, h, speed), action);
    }

    public Action maintainPositionAction() {
        return telemetry -> {
            dt.updatePose();
            dt.updateMovement();
            return false;
        };
    }

    public Action lowerArmAction() {
        return new WaitAction(2000);
    }

    public Action grabSpecimenAction() {
        return new WaitAction(2000);
    }

    public Action highBarAction() {
        return new WaitAction(2000);
    }

    public Action lowBarAction() {
        return new WaitAction(2000);
    }

    public Action hookSpecimenAction() {
        return new WaitAction(2000);
    }
    public Action grabSampleAction() {
        return new WaitAction(2000);
    }

    public Action highBasketAction() {
        return new WaitAction(2000);
    }

    public Action lowBasketAction() {
        return new WaitAction(2000);
    }

    public Action dropSampleAction() {
        return new WaitAction(2000);
    }
}
