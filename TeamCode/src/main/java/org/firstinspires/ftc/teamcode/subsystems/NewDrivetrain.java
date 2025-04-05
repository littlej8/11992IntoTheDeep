package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

// too many changes to keep track of so just using a new class with the bare minimum
@Config
public class NewDrivetrain implements Subsystem {
    private final DcMotorEx fl, fr, bl, br;
    private final GoBildaPinpointDriver odo;
    private double maxPower;
    private double linearFinishDist = 1.5, headingFinishDist = Math.toRadians(10);

    private Pose2d pose;
    private Pose2d velocity;
    private Pose2d target = new Pose2d(0, 0, 0);

    public static double xP = 0.1, xF = 0.2, xD = 0.02;
    public static double yP = 0.1, yF = 0.2, yD = 0.02;
    public static double hP = 2.0, hF = 0, hD = 0;

    private double lastHeadingErr = 0, lastXErr = 0, lastYErr = 0;
    private double lastUpdate = System.nanoTime();

    private double turnTarget = 0, lastTurnInput = 0; //for teleop heading pid

    public NewDrivetrain(HardwareMap hw, Pose2d start, double maxPower, boolean teleop) {
        this.maxPower = maxPower;

        //idk if this helps with odo bc it's i2c but im keeping it anyway
        for (LynxModule module : hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        fl = hw.get(DcMotorEx.class, "front_left_motor");
        fr = hw.get(DcMotorEx.class, "front_right_motor");
        bl = hw.get(DcMotorEx.class, "back_left_motor");
        br = hw.get(DcMotorEx.class, "back_right_motor");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hw.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-30, 124); //mm
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (!teleop) {
            odo.recalibrateIMU();
            odo.setPosition(new Pose2D(DistanceUnit.INCH, start.position.x, start.position.y, AngleUnit.RADIANS, start.heading.toDouble()));
        }

        pose = start;
        velocity = new Pose2d(0, 0, 0);
    }

    public NewDrivetrain(HardwareMap hw, Pose2d start, double maxPower) {
        this(hw, start, maxPower, false);
    }

    public NewDrivetrain(HardwareMap hw, double maxPower) {
        this(hw, new Pose2d(0, 0, 0), maxPower);
    }

    public NewDrivetrain(HardwareMap hw) {
        this(hw, new Pose2d(0, 0, 0), 1.0);
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setLinearFinishDist(double dist) {
        linearFinishDist = dist;
    }

    public void setHeadingFinishDist(double dist) {
        headingFinishDist = dist;
    }

    // proportional, derivative, and lower-limit (basically Ks)
    private double pdl(double error, double dErr, double dt, double Kp, double Kd, double Kl, double deadzone) {
        double p = (error * Kp);
        double d = (dErr / dt) * Kd;
        double l = (Math.abs(error) > deadzone) ? Math.signum(error) * Kl : 0;

        return p + d + l;
    }

    @Override
    public void update(Telemetry telemetry) {
        double cur = System.nanoTime();
        double dt = (cur - lastUpdate) / 1E9;
        if (dt > 0.1) dt = 0.1; // don't overreact
        lastUpdate = cur;

        odo.update();

        double xPos = odo.getPosition().getX(DistanceUnit.INCH);
        double yPos = odo.getPosition().getY(DistanceUnit.INCH);
        double hPos = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // just use last known point incremented by velocity if pinpoint returns nan or infinite heading
        if (!Double.isNaN(xPos) &&
            !Double.isNaN(yPos) &&
            !(Double.isNaN(hPos) || Math.abs(hPos) > Math.PI * 10)) {
            pose = new Pose2d(xPos, yPos, hPos);
        } else {
            // if bad data then estimate pose from velocity
            pose = new Pose2d(pose.position.x + velocity.position.x*dt, pose.position.y + velocity.position.y*dt, pose.heading.toDouble() + velocity.heading.toDouble()*dt);
        }

        double xVel = odo.getVelocity().getX(DistanceUnit.INCH);
        double yVel = odo.getVelocity().getY(DistanceUnit.INCH);
        double hVel = odo.getVelocity().getHeading(AngleUnit.RADIANS);
        if (!Double.isNaN(xVel) &&
                !Double.isNaN(yVel) &&
                !Double.isNaN(hVel)) {
            velocity = new Pose2d(xVel, yVel, hVel);
        }

        Vector2d driveVector = new Vector2d(
            target.position.x-pose.position.x,
            target.position.y-pose.position.y
        );
        Vector2d rotatedVector = new Vector2d(
            driveVector.x * Math.cos(-pose.heading.toDouble()) - driveVector.y * Math.sin(-pose.heading.toDouble()),
            driveVector.x * Math.sin(-pose.heading.toDouble()) + driveVector.y * Math.cos(-pose.heading.toDouble())
        );

        double hErr = AngleUnit.normalizeRadians(target.heading.toDouble() - pose.heading.toDouble());
        double turn = pdl(hErr, hErr - lastHeadingErr, dt, hP, hD, hF, Math.toRadians(5));
        double x = pdl(rotatedVector.x, rotatedVector.x - lastXErr, dt, xP, xD, xF, 0.25);
        double y = pdl(rotatedVector.y, rotatedVector.y - lastYErr, dt, yP, yD, yF, 0.25);

        lastHeadingErr = hErr;
        lastXErr = rotatedVector.x;
        lastYErr = rotatedVector.y;

        /*double turn = AngleUnit.normalizeRadians(target.heading.toDouble() - heading) * hP;
        double x = rotatedVector.x * xP;//pfdX(rotatedVector.x);
        double y = rotatedVector.y * yP;//-pfdY(rotatedVector.y);*/

        double[] powers = setWheelPowers(x, y, turn);

        telemetry.addData("Pose", "(%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("Velocity", "(%.2f, %.2f, %.2f)", velocity.position.x, velocity.position.y, Math.toDegrees(velocity.heading.toDouble()));
        telemetry.addData("Target Pose", "(%.2f, %.2f, %.2f)", target.position.x, target.position.y, Math.toDegrees(target.heading.toDouble()));
        telemetry.addData("Wheel Powers", "(%.2f, %.2f, %.2f, %.2f)", powers[0], powers[1], powers[2], powers[3]);
    }

    // +x = forward, +y = left, +turn = ccw
    public double[] setWheelPowers(double x, double y, double h) {
        if (Double.isNaN(x)) x = 0;
        if (Double.isNaN(y)) y = 0;
        if (Double.isNaN(h)) h = 0;

        x *= -1;
        h *= -1;
        double pfr = (x - y - h);
        double pfl = (x + y + h);
        double pbr = (x + y - h);
        double pbl = (x - y + h);

        double max = Math.max(
                1,
                Math.max(
                        Math.abs(pfl),
                        Math.max(
                                Math.abs(pfr),
                                Math.max(
                                        Math.abs(pbl),
                                        Math.abs(pbr)
                                ))));
        if (max > maxPower) {
            pfl = (pfl / max) * maxPower;
            pfr = (pfr / max) * maxPower;
            pbl = (pbl / max) * maxPower;
            pbr = (pbr / max) * maxPower;
        }

        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        return new double[]{pfl, pfr, pbl, pbr};
    }

    public void driveTeleOp(Gamepad driver) {
        double y = 0;
        if (driver.left_trigger > 0.2) {
            y += driver.left_trigger / 2;
        }
        if (driver.right_trigger > 0.2) {
            y -= driver.right_trigger / 2;
        }

        double x = -driver.left_stick_y;
        double turn = Math.pow(-driver.right_stick_x, 3) * 0.5;

        // heading pid so it goes in a straight line
        if (Math.abs(turn) < 0.1) {
            if (Math.abs(lastTurnInput) > 0.1)
                turnTarget = getHeading();

            turn = hP * AngleUnit.normalizeRadians(turnTarget - getHeading());
        }

        lastTurnInput = turn;
        setWheelPowers(x, y, turn);
    }

    public void kill() {
        setWheelPowers(0, 0, 0);
    }

    public boolean moveFinished() {
        double linearDistance = Math.hypot(
                target.position.x - pose.position.x,
                target.position.y - pose.position.y
        );
        double angularDistance = AngleUnit.normalizeRadians(
                Math.abs(
                        target.heading.toDouble() - pose.heading.toDouble()
                )
        );
        return linearDistance < linearFinishDist && angularDistance < headingFinishDist;
    }

    public boolean moveFinishedAndStopped() {
        double linearVel = Math.hypot(velocity.position.x, velocity.position.y);
        double angularVel = Math.abs(velocity.heading.toDouble());

        return moveFinished() && linearVel < 5.0 && angularVel < 0.5;
    }

    public double getX() {
        return pose.position.x;
    }

    public double getY() {
        return pose.position.y;
    }

    public double getHeading() {
        return pose.heading.toDouble();
    }
}