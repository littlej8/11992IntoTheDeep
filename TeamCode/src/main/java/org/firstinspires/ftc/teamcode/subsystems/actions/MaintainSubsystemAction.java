package org.firstinspires.ftc.teamcode.subsystems.actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class MaintainSubsystemAction implements Action {
    Subsystem subsystem;

    public MaintainSubsystemAction(Subsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public boolean run(Telemetry telemetry) {
        subsystem.update(telemetry);
        return false;
    }
}
