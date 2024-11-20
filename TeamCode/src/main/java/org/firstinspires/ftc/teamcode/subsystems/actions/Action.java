package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    Runs until the run method returns false
 */
public interface Action {
    boolean run(Telemetry telemetry);
}