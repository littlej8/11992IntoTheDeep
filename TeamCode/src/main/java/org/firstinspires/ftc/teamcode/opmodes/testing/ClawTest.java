package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Config
@TeleOp(name="Claw Testing", group="Dashboard Tests")
public class ClawTest extends LinearOpMode {
    public static double wrist = 0, bend = 0;
    public static boolean grip = true;

    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            claw.rotate(wrist);
            claw.bend(bend);
            if (grip) {
                claw.grip();
            } else {
                claw.drop();
            }
        }
    }
}
