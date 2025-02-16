package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;

@TeleOp
public class BackAndForthTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(0)), true);
        robot.dt.setDriveRelativeToStart(false);
        /*robot.schedule(
                robot.moveAction(0, -48, 0),
                robot.moveAction(0, 0, 0)
        );*/

        waitForStart();

        ElapsedTime timeToNext = new ElapsedTime();

        boolean forward = true;

        robot.dt.setTargetPose(new Pose2d(48, 0, 0));

        while (opModeIsActive()) {
            robot.dt.updatePose(telemetry);
            robot.dt.updateMovement(telemetry);

            if (robot.dt.moveFinished() && timeToNext.seconds() > 2) {
                forward = !forward;
                timeToNext.reset();
            }

            if (timeToNext.seconds() > 1) {
                robot.dt.setTargetPose(new Pose2d((forward ? 48 : 0), (forward ? 0 : 0), 0));
            }

            telemetry.update();
        }
    }
}
