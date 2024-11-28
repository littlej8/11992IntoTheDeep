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

        Drivetrain dt = new Drivetrain(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)));
        boolean gone = false;

        waitForStart();

        while (opModeIsActive()) {
            if (dt.moveFinished() && !gone) {
                sleep(500);
                dt.setTargetPose(new Pose2d(72, 0, Math.toRadians(-90)));
                gone = true;
            } else if (dt.moveFinished()) {
                sleep(500);
                dt.setTargetPose(new Pose2d(0, 0, Math.toRadians(-90)));
                gone = false;
            }
            dt.update(telemetry);
        }
    }
}
