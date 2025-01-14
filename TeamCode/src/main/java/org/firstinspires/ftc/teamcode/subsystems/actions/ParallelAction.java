package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

/*
    Runs all actions and stop when last one is done
 */
public class ParallelAction implements Action {
    ArrayList<Action> actions = new ArrayList<>();
    ArrayList<Boolean> finished = new ArrayList<>();

    public ParallelAction(Action... actions) {
        this.actions.addAll(Arrays.asList(actions));
        for (int i = 0; i < this.actions.size(); i++) {
            finished.add(false);
        }
    }

    @Override
    public boolean run(Telemetry telemetry) {
        if (!finished.contains(false))
            return false;

        for (int i = 0; i < actions.size(); i++)
            if (!actions.get(i).run(telemetry) && !finished.get(i))
                finished.set(i, true);

        return true;
    }
}
