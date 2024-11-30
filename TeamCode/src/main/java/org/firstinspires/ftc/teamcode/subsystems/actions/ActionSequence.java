package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    Action wrapper for an action scheduler
 */
public class ActionSequence implements Action{
    ActionScheduler scheduler;

    public ActionSequence(Action... actions) {
        scheduler = new ActionScheduler(actions);
    }

    @Override
    public boolean run(Telemetry telemetry) {
        scheduler.runNextAction(telemetry);
        return !scheduler.actionsDone();
    }
}
