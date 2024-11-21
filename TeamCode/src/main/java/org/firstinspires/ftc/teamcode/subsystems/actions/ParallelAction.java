package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

/*
    Runs until all actions are completed
 */
public class ParallelAction implements Action {
    ArrayList<Action> actions = new ArrayList<>();

    public ParallelAction(Action... actions) {
        this.actions.addAll(Arrays.asList(actions));
    }

    @Override
    public boolean run(Telemetry telemetry) {
        if (actions.isEmpty())
            return false;

        ArrayList<Action> actionsStillRunning = new ArrayList<>();

        for (int i = 0; i < actions.size(); i++)
            if (actions.get(i).run(telemetry))
                actionsStillRunning.add(actions.get(i));

        actions = actionsStillRunning;

        return true;
    }
}
