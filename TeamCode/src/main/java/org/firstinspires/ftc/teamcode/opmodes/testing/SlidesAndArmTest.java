package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionScheduler;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@Disabled
@Config
@TeleOp
public class SlidesAndArmTest extends LinearOpMode {
    public static double armTarget = -55;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionScheduler scheduler = new ActionScheduler();
        Slides slides = new Slides(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);

        scheduler.schedule(
                new WaitAction(2000),
                new UntilAction(arm.goToAction(90), new MaintainSubsystemAction(slides)),
                new WaitAction(1000),
                new UntilAction(slides.goToAction(25), new MaintainSubsystemAction(arm)),
                new WaitAction(1000),
                new UntilAction(slides.goToAction(0), new MaintainSubsystemAction(arm)),
                new WaitAction(1000),
                new UntilAction(arm.goToAction(-35), new MaintainSubsystemAction(slides))
        );

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int state = 0;

        while (opModeIsActive()) {
            if (timer.seconds() > 2 && state == 0) {
                arm.setTarget(90);
                state++;
            }
            if (timer.seconds() > 4 && state == 1) {
                slides.setTarget(25);
                state++;
            }
            if (timer.seconds() > 6 && state == 2) {
                slides.setTarget(0);
                state++;
            }
            if (timer.seconds() > 8 && state == 3) {
                arm.setTarget(-35);
                state++;
            }

            arm.update();
            slides.update();
            //scheduler.runNextAction(telemetry);
            telemetry.update();
        }
    }
}
