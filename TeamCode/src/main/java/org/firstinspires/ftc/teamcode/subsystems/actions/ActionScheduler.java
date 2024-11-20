package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NullTelemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class ActionScheduler {
    private ArrayList<Action> actionsStack = new ArrayList<>();

    public ActionScheduler() {}
    public ActionScheduler(Action... actions) {
        actionsStack.addAll(Arrays.asList(actions));
    }

    public void schedule(Action... actions) {
        actionsStack.addAll(Arrays.asList(actions));
    }

    public Action getNextAction() {
        return actionsStack.get(actionsStack.size()-1);
    }

    public void removeCurrentAction() {
        actionsStack.remove(actionsStack.size()-1);
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
