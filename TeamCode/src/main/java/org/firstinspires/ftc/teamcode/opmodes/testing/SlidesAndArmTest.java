package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@Config
@TeleOp
public class SlidesAndArmTest extends LinearOpMode {
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionScheduler scheduler = new ActionScheduler();
        Slides slides = new Slides(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);

        scheduler.schedule(
                new WaitAction(2000),
                arm.goToAction(Arm.Position.GRAB),
                new WaitAction(1000),
                slides.goToAction(20),
                new WaitAction(1000),
                slides.goToAction(0),
                new WaitAction(1000),
                arm.goToAction(Arm.Position.RETRACTED)
        );

        waitForStart();

        while (opModeIsActive()) {
            scheduler.runNextAction(telemetry);
            telemetry.update();
        }
    }
}
