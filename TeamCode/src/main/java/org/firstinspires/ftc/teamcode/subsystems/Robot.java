package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;

public class Robot implements Subsystem {
    ActionScheduler scheduler = new ActionScheduler();
    Telemetry telemetry;
    public Drivetrain dt;
    public Claw claw;
    public Arm arm;
    public Slides slides;

    public Robot(HardwareMap hw, Telemetry telemetry, Pose2d startPose) {
        this.telemetry = telemetry;
        dt = new Drivetrain(hw, telemetry, startPose);
        arm = new Arm(hw, telemetry);
        slides = new Slides(hw, telemetry);
        claw = new Claw(hw);
    }

    public Robot(HardwareMap hw, Telemetry telemetry) {
        this(hw, telemetry, new Pose2d(0, 0, 0));
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
            claw = new Claw(hw);
        }
    }

    public void schedule(Action... actions) {
        scheduler.schedule(actions);
    }

    public boolean actionsDone() {
        return scheduler.actionsDone();
    }

    public void update() {
        update(telemetry);
    }

    @Override
    public void update(Telemetry telemetry) {
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
            dt.updatePose(telemetry);
            dt.updateMovement(telemetry);
            return false;
        };
    }

    public Action armAction(double deg) {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(deg);
    }

    public Action slidesAction(double pos) {
        if (slides == null) {
            return new WaitAction(2000);
        }

        return slides.goToAction(pos);
    }

    public Action lowerArmAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.RETRACTED);
    }

    public Action armToGrabAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.GRAB);
    }

    public Action grabSpecimenAction() {
        if (claw == null) {
            return new WaitAction(2000);
        }

        return claw.gripAction();
    }

    public Action highBarAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.HIGH_HOOK);
    }

    public Action lowBarAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.LOW_HOOK);
    }

    public Action hookSpecimenAction() {
        if (arm == null || claw == null) {
            return new WaitAction(2000);
        }

        return new ActionSequence(
            arm.goToAction(arm.getArmPosition() - 5),
            claw.dropAction()
        );
    }

    public Action grabSampleAction() {
        if (claw == null) {
            return new WaitAction(2000);
        }

        return claw.gripAction();
    }

    public Action highBasketAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.HIGH_BASKET);
    }

    public Action lowBasketAction() {
        if (arm == null) {
            return new WaitAction(2000);
        }

        return arm.goToAction(Arm.Position.LOW_BASKET);
    }

    public Action dropSampleAction() {
        if (claw == null) {
            return new WaitAction(2000);
        }

        return claw.dropAction();
    }
}
