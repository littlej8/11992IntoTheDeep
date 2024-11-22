package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
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

    public Action grabSpecimenAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action highBarAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action lowBarAction() {
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

    public Action highBasketAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action lowBasketAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }

    public Action dropSampleAction() {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                return false;
            }
        };
    }
}
