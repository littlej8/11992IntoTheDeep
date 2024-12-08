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
public class Drivetrain implements Subsystem {
    DcMotorEx fl, fr, bl, br;
    IMU imu;
    double headingOffset;
    Pose2d pose, targetPose;

    Telemetry tel = null;

    public static double xP = 0.025, xI = 0.0, xD = 0.005;
    public static double yP = 0.025, yI = 0.0, yD = 0.005;
    public static double hP = 3, hI = 0, hD = 0.005;
    PIDController xPID = new PIDController(xP, xI, xD),
            yPID = new PIDController(yP, yI, yD),
            hPID = new PIDController(hP, hI, hD, true);

    public static double TICKS_PER_REV = 384.5; //145.1 if 1150rpm motors; 384.5 if 435rpm
    public static double WHEEL_DIAMETER = 96 / 25.4; //75mm if small black wheels; 96mm if big gray or yellow

    public static double IN_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;

    public static double FORWARD_GAIN = 1.063594;
    public static double STRAFE_GAIN = 1.268493;
    public static double MAX_WHEEL_POWER = 0.5;

    public static double LINEAR_FINISH_DIST = 1.0;
    public static double ANGULAR_FINISH_DIST = Math.toRadians(5.0);

    boolean driveRelativeToStart = true;
    Pose2d startPose = null;

    //fl, fr, bl, br
    double[] prevWheels = new double[]{0, 0, 0, 0}, wheels = new double[]{0, 0, 0, 0};
    double prevHeading;

    ElapsedTime timer;

    public Drivetrain(HardwareMap hw, Pose2d startPose) {
        this.startPose = startPose;
        pose = startPose;
        headingOffset = pose.heading.toDouble();
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

    public Drivetrain(HardwareMap hw, Telemetry t) {
        this(hw);
        tel = t;
    }

    public Drivetrain(HardwareMap hw, Telemetry t, Pose2d start) {
        this(hw, start);
        tel = t;
    }

    public void setDriveRelativeToStart(boolean driveRelativeToStart) {
        this.driveRelativeToStart = driveRelativeToStart;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d newPose) {
        pose = newPose;
    }

    public void setPose(double x, double y, double heading) {
        setPose(new Pose2d(x, y, heading));
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d targetPose) {
        if (driveRelativeToStart) {
            this.targetPose = new Pose2d(targetPose.position.plus(startPose.position), targetPose.heading);
        } else {
            this.targetPose = targetPose;
        }
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

    public void setX(double x) {
        setPose(x, getY(), getHeading());
    }

    public void setY(double y) {
        setPose(getX(), y, getHeading());
    }

    public void setHeading(double h) {
        headingOffset -= getHeading() - h;
        setPose(getX(), getY(), h);
    }

    public void updatePose(Telemetry telemetry) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset;
        wheels[0] = fl.getCurrentPosition();
        wheels[1] = fr.getCurrentPosition();
        wheels[2] = bl.getCurrentPosition();
        wheels[3] = br.getCurrentPosition();

        double dfl = (wheels[0] - prevWheels[0]) * IN_PER_TICK;
        double dfr = (wheels[1] - prevWheels[1]) * IN_PER_TICK;
        double dbl = (wheels[2] - prevWheels[2]) * IN_PER_TICK;
        double dbr = (wheels[3] - prevWheels[3]) * IN_PER_TICK;

        double dy = ((dfl + dfr + dbl + dbr) / 4) * STRAFE_GAIN;
        double dx = ((dbl + dfr - dfl - dbr) / 4) * FORWARD_GAIN;
        double dtheta = (heading - prevHeading);

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
        pose = new Pose2d(pose.position.x + twist.line.x, pose.position.y + twist.line.y, heading);

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

    public void killPowers() {
        setDrivePowers(0, 0, 0);
    }

    public double[] setDrivePowers(double x, double y, double h) {
        double pfl = x + y - h;
        double pfr = -x + y + h;
        double pbl = -x + y - h;
        double pbr = x + y + h;

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

        return new double[]{pfl, pfr, pbl, pbr};
    }

    public double[] setFieldPowers(double x, double y, double h) {
        Vector2d vec = new Vector2d(x, y);
        vec = rotateVec(vec, -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        return setDrivePowers(vec, h);
    }

    public double[] setDrivePowers(Vector2d linear, double h) {
        return setDrivePowers(linear.x, linear.y, h);
    }

    public static Vector2d rotateVec(Vector2d vec, double angle) {
        double newX = vec.x * Math.cos(angle) - vec.y * Math.sin(angle);
        double newY = vec.x * Math.sin(angle) + vec.y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }

    public void updateMovement(Telemetry telemetry) {
        xPID.setPID(xP, xI, xD);
        yPID.setPID(yP, yI, yD);
        hPID.setPID(hP, hI, hD);

        double xPower = xPID.update(pose.position.x, targetPose.position.x);
        double yPower = yPID.update(pose.position.y, targetPose.position.y);
        double hPower = hPID.update(pose.heading.toDouble(), targetPose.heading.toDouble());

        if (Math.abs(xPower) < 0.03)
            xPower = 0;
        if (Math.abs(yPower) < 0.03)
            yPower = 0;
        if (Math.abs(hPower) < 0.03)
            hPower = 0;

        Vector2d linearPowers = rotateVec(new Vector2d(xPower, yPower), -pose.heading.toDouble());

        double[] p = setDrivePowers(linearPowers.x, linearPowers.y, hPower);

        if (telemetry != null) {
            telemetry.addData("Target Pose", "(%.2f, %.2f, %.2f)", targetPose.position.x, targetPose.position.y, Math.toDegrees(targetPose.heading.toDouble()));
            telemetry.addData("Wheel Powers", "(%f, %f, %f, %f)", p[0], p[1], p[2], p[3]);
        }
    }

    public void updateMovement() {
        updateMovement(null);
    }

    @Override
    public void update(Telemetry telemetry) {
        updatePose(telemetry);
        updateMovement(telemetry);
    }

    public void update() {
        update(tel);
    }

    public void moveTo(Pose2d target, Telemetry telemetry) {
        targetPose = target;
        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            if (telemetry != null) {
                updatePose(telemetry);
                updateMovement(telemetry);
                debug(telemetry);
                telemetry.update();
            } else {
                updatePose();
                updateMovement();
            }
            running = !moveFinished();
        }
        killPowers();
    }

    public void moveTo(double x, double y, double h) {
        moveTo(new Pose2d(x, y, Math.toRadians(h)));
    }

    public void moveToWithSpeed(double x, double y, double h, double max_speed) {
        double tmp = MAX_WHEEL_POWER;
        MAX_WHEEL_POWER = max_speed;
        moveTo(x, y, h);
        MAX_WHEEL_POWER = tmp;
    }

    public void moveTo(Pose2d target) {
        moveTo(target, tel);
    }

    public void maintainPosition(double millis) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < millis && !Thread.currentThread().isInterrupted()) {
            updatePose(tel);
            updateMovement(tel);
            if (tel != null) {
                debug(tel);
            }
        }
    }

    public boolean moveFinished() {
        double linearDistance = Math.hypot(targetPose.position.x - pose.position.x, targetPose.position.y - pose.position.y);
        double angularDistance = AngleUnit.normalizeRadians(Math.abs(targetPose.heading.toDouble() - pose.heading.toDouble()));
        return linearDistance < LINEAR_FINISH_DIST && angularDistance < ANGULAR_FINISH_DIST;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Loop Time", "%f hz", 1 / timer.seconds());
        timer.reset();
    }
}
