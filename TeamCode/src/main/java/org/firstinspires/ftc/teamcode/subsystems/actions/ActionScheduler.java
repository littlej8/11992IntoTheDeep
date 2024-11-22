package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class ActionScheduler {
    // a queue is first in first out
    private final Queue<Action> actionsQueue = new LinkedList<>();

    public ActionScheduler() {}
    public ActionScheduler(Action... actions) {
        actionsQueue.addAll(Arrays.asList(actions));
    }

    public void schedule(Action... actions) {
        actionsQueue.addAll(Arrays.asList(actions));
    }

    public Action getNextAction() {
        if (!actionsQueue.isEmpty())
            return actionsQueue.peek();
        else
            return telemetry -> false;
    }

    public void removeCurrentAction() {
        if (!actionsQueue.isEmpty())
            actionsQueue.remove();
    }

    public void runNextAction(Telemetry telemetry) {
        if (!getNextAction().run(telemetry)) {
            removeCurrentAction();
        }
    }

    public void runNextAction() {
        runNextAction(new NullTelemetry());
    }
}
