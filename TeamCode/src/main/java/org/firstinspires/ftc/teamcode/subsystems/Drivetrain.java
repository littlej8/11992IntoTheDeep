package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Drivetrain {
    DcMotorEx fl, fr, bl, br;
    IMU imu;
    Pose2d pose, targetPose;

    public static double xP = 1, xI = 0, xD = 0;
    public static double yP = 1, yI = 0, yD = 0;
    public static double hP = 3, hI = 0, hD = 0;
    PIDController xPID = new PIDController(xP, xI, xD),
            yPID = new PIDController(yP, yI, yD),
            hPID = new PIDController(hP, hI, hD, true);

    public static double TICKS_PER_REV = 384.5; //145.1 if 1150rpm motors; 384.5 if 435rpm
    public static double WHEEL_DIAMETER = 96 / 25.4; //75mm if small black wheels; 96mm if big gray

    public static double IN_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;
    public static double LAT_IN_PER_TICK = IN_PER_TICK;

    public static double STRAFE_MULT = 1.0;//1.45;
    public static double FORWARD_MULT = 1.0;//1.525;
    public static double HEADING_MULT = 1.0;
    public static double MAX_WHEEL_POWER = 0.3;

    //fl, fr, bl, br
    double[] prevWheels = new double[]{0, 0, 0, 0}, wheels = new double[]{0, 0, 0, 0};
    double prevHeading;

    ElapsedTime timer;

    public Drivetrain(HardwareMap hw, Pose2d startPose) {
        pose = startPose;
        targetPose = startPose;
        prevHeading = pose.heading.toDouble();

        for (LynxModule module : hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        fl = hw.get(DcMotorEx.class, "front_left_motor");
        fr = hw.get(DcMotorEx.class, "front_right_motor");
        bl = hw.get(DcMotorEx.class, "back_left_motor");
        br = hw.get(DcMotorEx.class, "back_right_motor");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        timer = new ElapsedTime();
    }

    public Drivetrain(HardwareMap hw) {
        this(hw, new Pose2d(0, 0, 0));
    }

    public void updatePose(Telemetry telemetry) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        wheels[0] = fl.getCurrentPosition();
        wheels[1] = fr.getCurrentPosition();
        wheels[2] = bl.getCurrentPosition();
        wheels[3] = br.getCurrentPosition();

        double dfl = (wheels[0] - prevWheels[0]) * IN_PER_TICK;
        double dfr = (wheels[1] - prevWheels[1]) * IN_PER_TICK;
        double dbl = (wheels[2] - prevWheels[2]) * IN_PER_TICK;
        double dbr = (wheels[3] - prevWheels[3]) * IN_PER_TICK;

        double dy = ((dfl + dfr + dbl + dbr) / 4) * STRAFE_MULT;
        double dx = ((dbl + dfr - dfl - dbr) / 4) * FORWARD_MULT;
        double dtheta = (heading - prevHeading) * HEADING_MULT;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        // fancy math to apply the rotation midway through the change in position
        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        double xrot = dx * s - dy * c;
        double yrot = dx * c + dy * s;

        double curCos = Math.cos(-heading);
        double curSin = Math.sin(-heading);

        Twist2d twist = new Twist2d(
                new Vector2d(
                        -(xrot * curCos - yrot * curSin),
                        xrot * curSin + yrot * curCos),
                dtheta);
        pose = new Pose2d(pose.plus(twist).position, heading);

        prevWheels[0] = wheels[0];
        prevWheels[1] = wheels[1];
        prevWheels[2] = wheels[2];
        prevWheels[3] = wheels[3];
        prevHeading = heading;

        if (telemetry != null) {
            telemetry.addData("Current Pose", "(%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Velocity", "(%.2f, %.2f, %.2f)", twist.line.x, twist.line.y, Math.toDegrees(dtheta));
        }
    }

    public void updatePose() {
        updatePose(null);
    }

    public void updateMovement(Telemetry telemetry) {
        xPID.setPID(xP, xI, xD);
        yPID.setPID(yP, yI, yD);
        hPID.setPID(hP, hI, hD);

        double xPower = xPID.update(pose.position.x, targetPose.position.x);
        double yPower = yPID.update(pose.position.y, targetPose.position.y);
        double hPower = hPID.update(pose.heading.toDouble(), targetPose.heading.toDouble());

        if (Math.abs(xPower) < 0.01)
            xPower = 0;
        if (Math.abs(yPower) < 0.01)
            yPower = 0;
        if (Math.abs(hPower) < 0.01)
            hPower = 0;

        double x0 = xPower;
        double y0 = yPower;
        double cos = Math.cos(-pose.heading.toDouble());
        double sin = Math.sin(-pose.heading.toDouble());
        xPower = x0 * cos - y0 * sin;
        yPower = x0 * sin + y0 * cos;

        double pfl = xPower + yPower - hPower;
        double pfr = -xPower + yPower + hPower;
        double pbl = -xPower + yPower - hPower;
        double pbr = xPower + yPower + hPower;

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
        if (max > MAX_WHEEL_POWER) {
            pfl = (pfl / max) * MAX_WHEEL_POWER;
            pfr = (pfr / max) * MAX_WHEEL_POWER;
            pbl = (pbl / max) * MAX_WHEEL_POWER;
            pbr = (pbr / max) * MAX_WHEEL_POWER;
        }

        fl.setPower(pfl);
        fr.setPower(pfr);
        bl.setPower(pbl);
        br.setPower(pbr);

        if (telemetry != null) {
            telemetry.addData("Target Pose", "(%.2f, %.2f, %.2f)", targetPose.position.x, targetPose.position.y, Math.toDegrees(targetPose.heading.toDouble()));
            telemetry.addData("Wheel Powers", "(%f, %f, %f, %f)", pfl, pfr, pbl, pbr);
        }
    }

    public void updateMovement() {
        updateMovement(null);
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Loop Time", "%f hz", 1 / timer.seconds());
        timer.reset();
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }
}
