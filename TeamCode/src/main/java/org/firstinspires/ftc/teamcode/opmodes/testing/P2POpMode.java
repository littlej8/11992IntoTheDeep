package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Config
@TeleOp(name = "P2POpMode")
public class P2POpMode extends LinearOpMode {
    String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
    List<DcMotorEx> motors = new ArrayList<>();

    IMU imu;

    // TODO: tune based on the wheels and motors
    public static double TICKS_PER_REV = 384.5; //145.1 if 1150rpm motors; 384.5 if 435rpm
    public static double WHEEL_DIAMETER = 96 / 25.4; //2 if small black wheels; 4 if big gray

    public static double IN_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;

    public static double STRAFE_MULT = 1.0;//1.45;
    public static double FORWARD_MULT = 1.0;//1.525;
    public static double HEADING_MULT = 1.0;

    public static double TARGET_X = 0.0;
    public static double TARGET_Y = 24.0;
    public static double TARGET_HEADING = 180.0;

    public static double Px = 1.0;
    public static double Py = 1.0;
    public static double Ph = 3.0;
    public static double WHEEL_CACHE_MIN = 0.01;
    public static double MAX_WHEEL_POWER = 0.3;
    public static double MAX_WHEEL_ACCEL = 0.05; //power/sec

    public ElapsedTime programTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // activate bulk-reading for faster loop times
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // initialize motor array
        for (String motor : motorNames)
            motors.add(hardwareMap.get(DcMotorEx.class, motor));

        // initialize motors to run without encoder
        // (run without encoder just means it doesn't use the encoder to keep speeds consistent)
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // reverse the left side
        motors.get(0).setDirection(DcMotor.Direction.REVERSE);
        motors.get(1).setDirection(DcMotor.Direction.REVERSE);

        // initialize imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // 0: fl
        // 1: fr
        // 2: bl
        // 3: br
        double[] prevWheels = new double[]{0, 0, 0, 0}, wheels = new double[]{0, 0, 0, 0};
        double heading = 0, prevHeading = 0;
        double[] curPose = new double[]{0.0, 0.0, 0.0};
        double[] motorCache = new double[]{0.0, 0.0, 0.0, 0.0};

        waitForStart();

        double lastTime = programTimer.seconds();

        while (!isStopRequested()) {
            double timeTotal = programTimer.seconds(); // + Math.PI;

            /*double sinTime = Math.sin(timeTotal / 2);
            double cosTime = Math.cos(timeTotal / 2);

            double radius = 24;
            double xtarget = (cosTime * radius) - radius/2;
            double ytarget = (sinTime * radius);

            TARGET_X = xtarget;
            TARGET_Y = ytarget;
            TARGET_HEADING = Math.toDegrees(timeTotal / 2) - 90;*/

            //telemetry.addData("xtarget", xtarget);
            //telemetry.addData("ytarget", ytarget);

            /*if (timeTotal > 4) {
                TARGET_X = 24;
                TARGET_HEADING = 0;
            }
            if (timeTotal > 7) {
                TARGET_Y = 0;
                TARGET_HEADING = 180;
            }
            if (timeTotal > 10) {
                TARGET_X = 0;
                TARGET_HEADING = 0;
            }*/

            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // get the change in encoder position converted to inches
            wheels[0] = motors.get(1).getCurrentPosition();
            wheels[1] = motors.get(2).getCurrentPosition();
            wheels[2] = motors.get(0).getCurrentPosition();
            wheels[3] = motors.get(3).getCurrentPosition();

            double dfl = (wheels[0] - prevWheels[0]) * IN_PER_TICK;
            double dfr = (wheels[1] - prevWheels[1]) * IN_PER_TICK;
            double dbl = (wheels[2] - prevWheels[2]) * IN_PER_TICK;
            double dbr = (wheels[3] - prevWheels[3]) * IN_PER_TICK;

            // use forward kinematics to figure out how the robot moved based on the wheels
            double twistRobotY = (dfl + dfr + dbl + dbr) / 4;
            double twistRobotX = (dbl + dfr - dfl - dbr) / 4;
            double twistRobotTheta = heading - prevHeading;

            prevWheels[0] = wheels[0];
            prevWheels[1] = wheels[1];
            prevWheels[2] = wheels[2];
            prevWheels[3] = wheels[3];
            prevHeading = heading;

            // integrate the change into the current estimate in field coordinates
            double[] twist = new double[]{
                    twistRobotX * STRAFE_MULT,
                    twistRobotY * FORWARD_MULT,
                    twistRobotTheta * HEADING_MULT
            };

            exp(curPose, twist);
            curPose[2] = heading;

            // get field oriented x, y, and turn powers through a p controller
            double xPower = (TARGET_X - curPose[0]) * Px;
            double yPower = (TARGET_Y - curPose[1]) * Py;
            double headingError = angleWrap(Math.toRadians(TARGET_HEADING) - curPose[2]);
            double hPower = headingError * Ph;

            // if the power is negligable, set it to 0
            if (Math.abs(xPower) < 0.01)
                xPower = 0;
            if (Math.abs(yPower) < 0.01)
                yPower = 0;
            if (Math.abs(hPower) < 0.01)
                hPower = 0;

            // rotate the x and y powers by the opposite of the robot's heading
            double x0 = xPower;
            double y0 = yPower;
            double cos = Math.cos(-curPose[2]);
            double sin = Math.sin(-curPose[2]);
            xPower = x0 * cos - y0 * sin;
            yPower = x0 * sin + y0 * cos;

            // calculate individual wheel powers using inverse kinematics
            double[] p = new double[4];
            p[0] = -xPower + yPower - hPower;
            p[1] = xPower + yPower - hPower;
            p[2] = -xPower + yPower + hPower;
            p[3] = xPower + yPower + hPower;

            // get the largest power and constrain it to [-1,1] while keeping the others proportional
            double max = Math.max(
                    1,
                    Math.max(
                            Math.abs(p[0]),
                            Math.max(
                                    Math.abs(p[1]),
                                    Math.max(
                                            Math.abs(p[2]),
                                            Math.abs(p[3])
                                    ))));
            if (max > MAX_WHEEL_POWER)
                for (int i = 0; i < 4; i++)
                    p[i] = (p[i] / max) * MAX_WHEEL_POWER;

            double dt = timeTotal - lastTime;

            // set motor powers only if the difference is noticable or sets to 0
            for (int i = 0; i < 4; i++) {
                if (Math.abs(p[i]) <= WHEEL_CACHE_MIN) {
                    p[i] = 0;
                } else if (Math.abs(motorCache[i] - p[i]) > WHEEL_CACHE_MIN) {
                    if (Math.abs(motorCache[i] - p[i]) * dt >= MAX_WHEEL_ACCEL)
                        p[i] = motorCache[i] + (MAX_WHEEL_ACCEL * ((p[i] < 0) ? -1 : 1));
                    motors.get(i).setPower(p[i]);
                    motorCache[i] = p[i];
                }
            }

            // log all data
            telemetry.addData("Current Pose", "(%.2f, %.2f, %.2f)", curPose[0], curPose[1], Math.toDegrees(curPose[2]));
            telemetry.addData("Target Pose", "(%.2f, %.2f, %.2f)", TARGET_X, TARGET_Y, TARGET_HEADING);
            telemetry.addData("Velocity", "(%.2f, %.2f, %.2f)", twist[0], twist[1], Math.toDegrees(twistRobotTheta));
            telemetry.addData("Loop Time", "%f hz", 1 / dt);
            telemetry.addData("Wheel Powers", "(%f, %f, %f, %f)", p[0], p[1], p[2], p[3]);
            telemetry.update();

            lastTime = timeTotal;
        }
    }

    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    // integrate the last change into the total estimate using the constant acceleration formula
    public static void exp(double[] cur, double[] twist) {
        double dx = twist[0];
        double dy = twist[1];
        double dtheta = twist[2];

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        double[] transform = new double[]{
                dx * s - dy * c,
                dx * c + dy * s,
        };

        double curCos = Math.cos(-cur[2]);
        double curSin = Math.sin(-cur[2]);

        cur[0] -= transform[0] * curCos - transform[1] * curSin;
        cur[1] += transform[0] * curSin + transform[1] * curCos;
    }
}