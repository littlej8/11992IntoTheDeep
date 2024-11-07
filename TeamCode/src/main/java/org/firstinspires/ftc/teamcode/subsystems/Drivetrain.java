package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Drivetrain {
    DcMotorEx fl, fr, bl, br;
    IMU imu;

    PIDController xPID, yPID, hPID;

    public static double TICKS_PER_REV = 384.5; //145.1 if 1150rpm motors; 384.5 if 435rpm
    public static double WHEEL_DIAMETER = 96 / 25.4; //2 if small black wheels; 4 if big gray

    public static double IN_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;
    public static double LAT_IN_PER_TICK = IN_PER_TICK;

    public Drivetrain(HardwareMap hw) {

    }
}
