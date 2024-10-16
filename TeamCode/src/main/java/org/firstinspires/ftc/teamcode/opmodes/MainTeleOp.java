package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.MecanumDrive;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (!isStopRequested()) {
            Vector2d linear = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double angular = gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(linear, angular));
        }
    }
}
