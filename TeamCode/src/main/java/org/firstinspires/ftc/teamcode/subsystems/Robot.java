package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionLocalizer;
import org.firstinspires.ftc.teamcode.util.BezCurve;
import org.firstinspires.ftc.teamcode.util.MotionProfile2d;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;

@Config
public class Robot implements Subsystem {
    ActionScheduler scheduler = new ActionScheduler();
    Telemetry telemetry;
    public Drivetrain dt;
    public VisionLocalizer vision;
    public Claw claw;
    public Arm arm;
    public Slides slides;

    public static boolean USE_MOTION_PROFILE = true;
    public static double MAX_VEL = 48, MAX_ACCEL = 48;

    public Robot(HardwareMap hw, Telemetry telemetry, Pose2d startPose) {
        this.telemetry = telemetry;
        dt = new Drivetrain(hw, telemetry, startPose);
        vision = new VisionLocalizer(hw);
        arm = new Arm(hw, telemetry);
        slides = new Slides(hw, telemetry);
        claw = new Claw(hw);

        /*hw.get(DcMotorEx.class, "hang1").setPower(0.07); //left
        hw.get(DcMotorEx.class, "hang2").setPower(0.09); //right*/
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
            vision = new VisionLocalizer(hw);
        } else {
            arm = new Arm(hw, telemetry);
            slides = new Slides(hw, telemetry);
            claw = new Claw(hw);
        }
    }

    public Robot(HardwareMap hw, Telemetry telemetry, Pose2d startPose, int throwaway) {
        this.telemetry = telemetry;
        dt = new Drivetrain(hw, telemetry, startPose);
        vision = new VisionLocalizer(hw);
        arm = new Arm(hw, telemetry);
        slides = new Slides(hw, telemetry);
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
        vision.updateVisionPose();
        Pose2d latestPose = vision.getLatestPose();
        if (latestPose != null) {
            telemetry.addData("Latest Vision Pose", "%6.1f, %6.1f, %6.1f", latestPose.position.x, latestPose.position.y, latestPose.heading.toDouble());
            telemetry.addData("Time since last vision pose", System.currentTimeMillis() - vision.getTimeStamp());
        }
        if (actionsDone()) {
            dt.updatePose(telemetry);
            drawRobot();
        } else {
            scheduler.runNextAction(telemetry);
        }
    }

    public void updateWithVision(Telemetry telemetry) {
        vision.updateVisionPose();
        Pose2d latestPose = vision.getLatestPose();
        if (latestPose != null) {
            if (vision.latestPoseValid() && Math.abs(dt.velocity.position.x) < 5 && Math.abs(dt.velocity.position.y) < 5 && Math.abs(dt.velocity.heading.toDouble()) < Math.toRadians(10)) {
                dt.addVisionUpdate(latestPose, vision.getTimeStamp());
            }
            telemetry.addData("Latest Vision Pose", "%6.1f, %6.1f, %6.1f", latestPose.position.x, latestPose.position.y, latestPose.heading.toDouble());
            telemetry.addData("Time since last vision pose", System.currentTimeMillis() - vision.getTimeStamp());
        }
        if (actionsDone()) {
            dt.updatePose(telemetry);
            drawRobot();
        } else {
            scheduler.runNextAction(telemetry);
        }
    }

    public Action moveAction(double x, double y, double h, double speed) {
        return new Action() {
            boolean init = false;
            final double prevPower = Drivetrain.MAX_WHEEL_POWER;
            MotionProfile2d profile;
            Vector2d initialPose;
            boolean onlyX = false;
            boolean onlyY = false;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    initialPose = new Vector2d(dt.getX(), dt.getY());
                    Drivetrain.MAX_WHEEL_POWER = speed;
                    dt.setTargetPose(new Pose2d(x, y, Math.toRadians(h)));
                    profile = new MotionProfile2d(new Vector2d(dt.getX(), dt.getY()), new Vector2d(x, y), MAX_VEL, MAX_ACCEL);
                    init = true;
                    if (Math.abs(dt.getX() - x) < 0.05) {
                        onlyX = true;
                    }
                    if (Math.abs(dt.getY() - y) < 0.05) {
                        onlyY = true;
                    }
                }

                Vector2d update = profile.update();
                if (USE_MOTION_PROFILE) {
                    if (onlyX) {
                        dt.setTargetPose(new Pose2d(x, profile.update().y, Math.toRadians(h)));
                    } else if (onlyY) {
                        dt.setTargetPose(new Pose2d(profile.update().x, y, Math.toRadians(h)));
                    } else {
                        dt.setTargetPose(new Pose2d(profile.update(), Math.toRadians(h)));
                    }
                }

                if (profile.finished() && dt.moveFinished()) {
                    Drivetrain.MAX_WHEEL_POWER = prevPower;
                    dt.killPowers();
                    return false;
                }

                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                TelemetryPacket p = new TelemetryPacket(true);
                Canvas c = p.fieldOverlay();

                c.setStroke("#4CAF50FF");
                c.setStrokeWidth(1);
                c.strokeLine(initialPose.x, initialPose.y, x, y);

                drawRobot(c);

                c.setStroke("#FF0000");
                c.setStrokeWidth(1);
                c.strokeCircle(update.x, update.y, 9);

                FtcDashboard.getInstance().sendTelemetryPacket(p);

                return true;
            }
        };
    }

    public void drawRobot() {
        TelemetryPacket p = new TelemetryPacket(true);
        drawRobot(p.fieldOverlay());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    public void drawRobot(Canvas c) {
        c.setStroke("#3F51B5");
        c.setStrokeWidth(1);
        c.strokeCircle(dt.getX(), dt.getY(), 9);

        Vector2d halfv = Rotation2d.fromDouble(dt.getHeading() + Math.PI/2).vec().times(0.5 * 9);
        Vector2d p1 = new Vector2d(dt.getX(), dt.getY()).plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public Action moveAction(double x, double y, double h) {
        return moveAction(x, y, h, Drivetrain.MAX_WHEEL_POWER);
    }

    public Action curveAction(Vector2d control1, Vector2d control2, Vector2d end, double heading) {
        return new Action() {
            boolean init = false;
            BezCurve curve;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    init = true;
                    curve = new BezCurve(dt.getPose().position, control1, control2, end, 48, 60);
                }

                dt.setTargetPose(new Pose2d(curve.update(), Math.toRadians(heading)));
                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                TelemetryPacket p = new TelemetryPacket(true);
                Canvas c = p.fieldOverlay();
                curve.draw(c);
                dt.draw(c);

                FtcDashboard.getInstance().sendTelemetryPacket(p);

                return !curve.finished();
            }
        };
    }

    public Action curveAction(double x, double y, double heading) {
        return curveAction(x, y, heading, false);
    }

    public Action curveAction(double x, double y, double heading, boolean yFirst) {
        return new Action() {
            boolean init = false;
            BezCurve curve;

            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    init = true;
                    curve = new BezCurve(dt.getPose().position, new Vector2d(x, y), 48, 60);
                    if (yFirst) {
                        curve.setYFirst();
                    }
                }

                dt.setTargetPose(new Pose2d(curve.update(), Math.toRadians(heading)));
                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                TelemetryPacket p = new TelemetryPacket(true);
                Canvas c = p.fieldOverlay();
                curve.draw(c);
                dt.draw(c);

                FtcDashboard.getInstance().sendTelemetryPacket(p);

                return !curve.finished();
            }
        };
    }

    public Action turnToAction(double h) {
        return telemetry -> {
            dt.setTargetPose(new Pose2d(dt.getX(), dt.getY(), Math.toRadians(h)));
            dt.updatePose(telemetry);
            dt.updateMovement(telemetry);

            if (Math.abs(h - Math.toDegrees(dt.getHeading())) < 15) {
                dt.killPowers();
                return false;
            }

            return true;
        };
    }

    public Action turnToAction(double h, double millis) {
        return new Action() {
            boolean init = false;
            ElapsedTime timer;
            double start; //radians
            @Override
            public boolean run(Telemetry telemetry) {
                if (!init) {
                    init = true;
                    timer = new ElapsedTime();
                    start = Math.toDegrees(dt.getHeading());
                }

                double progress = timer.milliseconds() / millis;
                double curTarget = start + (h - start) * progress;
                dt.setTargetPose(new Pose2d(dt.getX(), dt.getY(), Math.toRadians(curTarget)));
                dt.updatePose(telemetry);
                dt.updateMovement(telemetry);

                if (Math.abs(h - Math.toDegrees(dt.getHeading())) < 5) {
                    dt.killPowers();
                    return false;
                }

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
            dt.updateMovement(telemetry, true);
            drawRobot();
            return false;
        };
    }

    public Action relocalizeAction(Pose2d newPose) {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                dt.setPose(newPose);
                return false;
            }
        };
    }

    public Action relocalizeAction(Vector2d newLinear) {
        return new Action() {
            @Override
            public boolean run(Telemetry telemetry) {
                dt.setPose(new Pose2d(newLinear, dt.getHeading()));
                return false;
            }
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
        return new ParallelAction(
                new ParallelAction(claw.rotateAction(90), claw.bendAction(0), claw.gripAction()),
                new ActionSequence(
                        new WaitAction(1000),
                        armAction(-30) // stop if takes longer than 3 secs
                )
        );
    }

    public Action retractFromBasketAction() {
        Arm.MAX_POWER = 0.7;
        return new ActionSequence(
                new ParallelAction(claw.bendAction(120), claw.dropAction(), claw.rotateAction(90), new MaintainSubsystemAction(arm), new MaintainSubsystemAction(slides)),
                new ParallelAction(slidesAction(0), new MaintainSubsystemAction(arm))
        );
    }



    private double fudgeCount = 0;
    public Action armToGrabSpecAction() {
        fudgeCount++;
        return new ParallelAction(
                armAction((fudgeCount == 1) ? -6 : (fudgeCount == 2) ? -12 : -14),
                new ActionSequence(
                        new WaitAction(500),
                        new ParallelAction(claw.rotateAction(90), claw.bendAction(240), claw.dropAction())
                )
        );
    }

    public Action prepareToHookAction() {
        fudgeCount++;
        return new ParallelAction(
                armAction(((fudgeCount == 1) ? 23 : (fudgeCount == 2) ? 23 : 20)),
                new ActionSequence(
                        new WaitAction(500),
                        new ParallelAction(claw.rotateAction(90), claw.bendAction(270))
                )
        );
    }

    public Action lowBarAction() {
        return arm.goToAction(Arm.Position.LOW_HOOK);
    }

    public Action armToGrabSampleAction() {
        return new ParallelAction(
                armAction(-32),
                claw.rotateAction(90),
                claw.bendAction(140),
                claw.dropAction()
        );
    }

    public Action highBasketAction() {
        //Arm.MAX_POWER = 1.0;
        return new ActionSequence(
                armAction(70),
                new UntilAction(new WaitAction(500), new MaintainSubsystemAction(arm)),
                new UntilAction(slidesAction(24), new MaintainSubsystemAction(arm)),
                new UntilAction(new ActionSequence(
                        new WaitAction(500),
                        new ParallelAction(claw.rotateAction(90), claw.bendAction(200))
                ), new ParallelAction(new MaintainSubsystemAction(arm), new MaintainSubsystemAction(slides)))
        );
    }

    public Action lowBasketAction() {
        return new ActionSequence(
                armAction(60),
                new UntilAction(new ActionSequence(
                        new WaitAction(500),
                        new ParallelAction(claw.rotateAction(90), claw.bendAction(240))
                ), new MaintainSubsystemAction(arm))
        );
    }
}
