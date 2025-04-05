package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift implements Subsystem {
    DcMotorEx left, right;

    public Lift(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "left_lift");
        right = hw.get(DcMotorEx.class, "right_lift");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPowers(double p) {
        setLeftPower(p);
        setRightPower(p);
    }

    public void setLeftPower(double p) {
        left.setPower(p);
    }

    public void setRightPower(double p) {
        right.setPower(p);
    }

    public double getCurrentAvg() {
        return (getLeftCurrent() + getRightCurrent()) / 2.0;
    }

    public double getLeftCurrent() {
        return left.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getRightCurrent() {
        return right.getCurrent(CurrentUnit.MILLIAMPS);
    }

    @Override
    public void update(Telemetry telemetry) {

    }
}
