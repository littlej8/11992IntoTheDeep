package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.util.MotionProfile2d;

// disgusting slop in the other robot class so totally new one, plus new design calls for new code
public class NewRobot implements Subsystem {
    ActionScheduler scheduler = new ActionScheduler();
    public NewDrivetrain dt;
    public Claw claw;
    public NewArm arm;
    public NewSlides slides;

    public NewRobot(HardwareMap hw, Pose2d start, double speed, boolean teleop) {
        dt = new NewDrivetrain(hw, start, speed, teleop);
        claw = new Claw(hw); // servo so no reset
        arm = new NewArm(hw, teleop);
        slides = new NewSlides(hw); // servo so no reset
    }

    public NewRobot(HardwareMap hw, Pose2d start, double speed) {
        this(hw, start, speed, false);
    }

    // start pose is ignored when teleop is true
    public NewRobot(HardwareMap hw, double speed, boolean teleop) {
        this(hw, new Pose2d(0, 0, 0), speed, teleop);
    }

    public NewRobot(HardwareMap hw, double speed) {
        this(hw, new Pose2d(0, 0, 0), speed);
    }

    public NewRobot(HardwareMap hw) {
        this(hw, new Pose2d(0, 0, 0), 0.8);
    }

    public void schedule(Action... actions) {
        scheduler.schedule(actions);
    }

    public boolean actionsDone() {
        return scheduler.actionsDone();
    }

    @Override
    public void update(Telemetry telemetry) {
        if (actionsDone()) {
            dt.update(telemetry);
            drawRobot();
        } else {
            scheduler.runNextAction(telemetry);
        }
    }

    public void drawRobot() {
        TelemetryPacket p = new TelemetryPacket(true);
        drawRobot(p.fieldOverlay());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    public void drawRobot(Canvas c) {
        c.setStroke("#3F51B5");
        c.setStrokeWidth(1);
        c.strokeCircle(dt.getY(), dt.getX(), 9);

        Vector2d halfv = Rotation2d.fromDouble(dt.getHeading() + Math.PI/2).vec().times(0.5 * 9);
        Vector2d p1 = new Vector2d(dt.getY(), dt.getX()).plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public Action moveAction(double x, double y, double h, double speed) {
        return new Action() {
            boolean init = false;
            final double prevPower = dt.getMaxPower();
            Vector2d initialPose;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    initialPose = new Vector2d(dt.getX(), dt.getY());
                    dt.setMaxPower(speed);
                    dt.setTarget(new Pose2d(x, y, Math.toRadians(h)));
                    init = true;
                }

                if (dt.moveFinished()) {
                    dt.setMaxPower(prevPower);
                    dt.kill();
                    return false;
                }

                dt.update(telemetry);

                TelemetryPacket p = new TelemetryPacket(true);
                Canvas c = p.fieldOverlay();

                c.setStroke("#4CAF50FF");
                c.setStrokeWidth(1);
                c.strokeLine(initialPose.x, initialPose.y, x, y);

                drawRobot(c);

                FtcDashboard.getInstance().sendTelemetryPacket(p);

                return true;
            }
        };
    }

    public Action moveAction(double x, double y, double h) {
        return moveAction(x, y, h, dt.getMaxPower());
    }

    public Action moveAndAction(double x, double y, double h, Action action) {
        return new UntilAction(moveAction(x, y, h), action);
    }

    public Action moveAndAction(double x, double y, double h, double speed, Action action) {
        return new UntilAction(moveAction(x, y, h, speed), action);
    }

    public Action waitAction(double millis) {
        return new ActionSequence(
            new UntilAction(new WaitAction(millis), new ParallelAction(
                new MaintainSubsystemAction(dt),
                new MaintainSubsystemAction(claw),
                new MaintainSubsystemAction(arm),
                new MaintainSubsystemAction(slides)
            )),
            telemetry -> {
                dt.kill();
                arm.kill();
                return false;
            }
        );
    }

    public Action killAction() {
        return telemetry -> {
            dt.kill();
            arm.kill();
            return false;
        };
    }

    public Action armAction(double deg) {
        return telemetry -> {
            if (arm.getTarget() != deg)
                arm.setTarget(deg);
            arm.update(telemetry);

            if (arm.moveFinished()) {
                arm.motor.setPower(0);
                return false;
            }
            return true;
        };
    }
}