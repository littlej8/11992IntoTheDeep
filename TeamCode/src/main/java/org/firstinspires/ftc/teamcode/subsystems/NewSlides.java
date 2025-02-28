package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// slides use a servo now
@Config
public class NewSlides implements Subsystem {
    private Servo servo;

    final double totalRotation = 1.57 * (1800.0 / 2.0); // deg per microsecond * total microseconds (half cus start at 0)
    final double pulleyCircum = 1.456 * Math.PI; // gobilda pulley is 1.456 inches inner diameter
    final double totalLinearTravel = pulleyCircum * totalRotation;
    final double secPerInch = 1.68 / pulleyCircum; // 1.68 secs per rotation and circum is inches per rotation

    private double positionEstimate = 0, target = 0;
    private ElapsedTime timer;
    private double lastUpdate = 0;

    public NewSlides(HardwareMap hw) {
        servo = hw.get(Servo.class, "slides");
    }

    public void setTarget(double inches) {
        target = inches;

        //double normalizedPosition = ((inches * 2) / totalLinearTravel) - 1; // inches to [-1, 1]
        double normalizedPosition = inches / totalLinearTravel; // inches to [0, 1]
        servo.setPosition(-normalizedPosition); // counterclockwise is better for some reason
    }

    public double getTarget() {
        return target;
    }

    public double getPositionEstimate() {
        return positionEstimate;
    }

    // updates the estimated position to tell when target is reached
    @Override
    public void update(Telemetry telemetry) {
        double dt = timer.seconds() - lastUpdate;
        lastUpdate = timer.seconds();

        positionEstimate += (secPerInch * dt) * Math.signum(target - positionEstimate);
        if (Math.abs(target - positionEstimate) < 0.5) {
            positionEstimate = target;
        }

        telemetry.addData("slides position", positionEstimate);
        telemetry.addData("slides target", target);
    }

    public boolean atPosition() {
        return positionEstimate == target;
    }
}
