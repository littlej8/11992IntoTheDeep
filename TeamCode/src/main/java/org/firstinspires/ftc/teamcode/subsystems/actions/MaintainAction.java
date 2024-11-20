package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    Continuously runs another action until one action finishes
 */
public class MaintainAction implements Action {
    private Action checker;
    private Action runner;

    public MaintainAction(Action checker, Action runner) {
        this.checker = checker;
        this.runner = runner;
    }

    @Override
    public boolean run(Telemetry telemetry) {
        if (!checker.run(telemetry))
            return false;

        runner.run(telemetry);
        return true;
    }
}
