package org.firstinspires.ftc.teamcode.subsystems.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    Waits the specified amount of time
 */
public class WaitAction implements Action {
    ElapsedTime timer;
    double millis;
    boolean init = false;

    public WaitAction(double millis) {
        this.millis = millis;
    }

    @Override
    public boolean run(Telemetry telemetry) {
        if (!init) {
            timer = new ElapsedTime();
            init = true;
        }

        return timer.milliseconds() < millis;
    }
}
