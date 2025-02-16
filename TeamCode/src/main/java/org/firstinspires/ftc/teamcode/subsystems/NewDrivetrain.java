package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

// too many changes to keep track of so just using a new class with the bare minimum
public class NewDrivetrain implements Subsystem {
    private final DcMotorEx fl, fr, bl, br;
    private final GoBildaPinpointDriver odo;
    private double maxPower;
    private double linearFinishDist = 1, headingFinishDist = Math.toRadians(5);

    private Pose2d target = new Pose2d(0, 0, 0);

    public static double xP = 0.2, xF = 0.0, xD = 0;
    public static double yP = 0.2, yF = 0.0, yD = 0;
    public static double hP = 6.0, hI = 0, hD = 0;
    private double hErrSum = 0;
    private double lastHeadingErr = 0, lastXErr = 0, lastYErr = 0;

    public NewDrivetrain(HardwareMap hw, Pose2d start, double maxPower) {
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
        odo.setOffsets(30, -120); //mm
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, start.position.x, start.position.y, AngleUnit.RADIANS, start.heading.toDouble()));
    }

    public NewDrivetrain(HardwareMap hw, double maxPower) {
        this(hw, new Pose2d(0, 0, 0), maxPower);
    }

    public NewDrivetrain(HardwareMap hw) {
        this(hw, new Pose2d(0, 0, 0), 0.5);
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

    private double pidHeading(double error) {
        hErrSum += error;
        double derivative = error - lastHeadingErr;
        if (error > Math.PI) {
            error -= Math.PI;
        } else if (error < -Math.PI) {
            error += Math.PI;
        }
        double correction = (error * hP) + (hErrSum * hI) + (derivative * hD);
        lastHeadingErr = error;
        return correction;
    }

    double pfdX(double error) {
        double derivative = error - lastXErr;
        double correction = (error * xP) + (derivative * xD);
        correction += Math.signum(error) * xF;
        lastXErr = error;
        return correction;
    }

    double pfdY(double error) {
        double derivative = error - lastYErr;
        double correction = (error * yP) + (derivative * yD);
        correction += Math.signum(error) * yF;
        lastYErr = error;
        return correction;
    }

    @Override
    public void update(Telemetry telemetry) {
        odo.update();

        Vector2d driveVector = new Vector2d(
            target.position.x-odo.getPosition().getX(DistanceUnit.INCH),
            target.position.y-odo.getPosition().getY(DistanceUnit.INCH)
        );
        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        Vector2d rotatedVector = new Vector2d(
            driveVector.x * Math.cos(heading) - driveVector.y * Math.sin(heading),
            driveVector.x * Math.sin(heading) + driveVector.y * Math.cos(heading)
        );

        double turn = pidHeading(target.heading.toDouble() - heading);
        double x = pfdX(rotatedVector.x);
        double y = pfdY(rotatedVector.y);

        fr.setPower((x - y - turn) * maxPower);
        fl.setPower((x + y + turn) * maxPower);
        br.setPower((x + y - turn) * maxPower);
        bl.setPower((x - y + turn) * maxPower);
    }

    public void setWheelPowers(double x, double y, double h) {
        fr.setPower((x - y - h) * maxPower);
        fl.setPower((x + y + h) * maxPower);
        br.setPower((x + y - h) * maxPower);
        bl.setPower((x - y + h) * maxPower);
    }

    public void kill() {
        setWheelPowers(0, 0, 0);
    }

    public boolean moveFinished() {
        double linearDistance = Math.hypot(
                target.position.x - odo.getPosition().getX(DistanceUnit.INCH),
                target.position.y - odo.getPosition().getY(DistanceUnit.INCH)
        );
        double angularDistance = AngleUnit.normalizeRadians(
                Math.abs(
                        target.heading.toDouble() - odo.getPosition().getHeading(AngleUnit.RADIANS)
                )
        );
        return linearDistance < linearFinishDist && angularDistance < headingFinishDist;
    }

    public double getX() {
        return odo.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return odo.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return odo.getPosition().getHeading(AngleUnit.RADIANS);
    }
}