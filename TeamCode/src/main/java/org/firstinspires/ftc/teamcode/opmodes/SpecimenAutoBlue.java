package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Config
@Autonomous(name = "Specimen Auto Blue", preselectTeleOp = "MainTeleOp")
public class SpecimenAutoBlue extends LinearOpMode {
    public static boolean USE_VISION_LOCALIZATION = false;
    public static double MAX_SPEED = 0.8;

    Robot robot;

    // ram into the wall to reset position then grab
    private Action grabAction() {
        return new ActionSequence(
                robot.moveAndAction(-41, 62, 0, robot.armToGrabAction()),
                new UntilAction(
                    robot.grabSpecimenAction(),
                    robot.maintainPositionAction()
                )
        );
    }

    /*
    START WITH CAMERA CENTERED WITH TAG
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(-8, 64, Math.toRadians(180)), 69);
        robot.dt.setDriveRelativeToStart(false);
        double prevMaxWheelPower = Drivetrain.MAX_WHEEL_POWER;
        Drivetrain.MAX_WHEEL_POWER = MAX_SPEED;

        robot.schedule(
                robot.moveAndAction(-8, 40, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-40, 46, 180, robot.lowerArmAction()),
                robot.moveAction(-40, 4, 180),
                robot.turnToAction(-90),
                robot.moveAction(-58, 4, -90),
                robot.moveAction(-58, 56, -90),
                robot.turnToAction(0),
                robot.moveAndAction(-41, 51, 0, robot.armToGrabAction()),
                robot.relocalizeAction(new Vector2d(-41, 48)),
                grabAction(),
                robot.moveAction(-41, 51, 0),
                robot.turnToAction(180),
                robot.moveAndAction(-6, 28, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-41, 51, 180, robot.armToGrabAction()),
                robot.turnToAction(0),
                robot.relocalizeAction(new Vector2d(-38, 50)),
                grabAction(),
                robot.moveAction(-41, 51, 0),
                robot.turnToAction(180),
                robot.moveAndAction(-4, 28, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-62, 64, 180, robot.lowerArmAction())
        );

        /*
        robot.schedule(
                robot.moveAndAction(0, 30, 180, 0.3, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 18, 180, robot.lowerArmAction()),
                robot.moveAction(32, 60, 180),
                robot.moveAction(50, 60, -90),
                robot.moveAction(50, 8, -90, 0.4),
                //robot.relocalizeAction(new Vector2d(13, -52)),
                robot.moveAndAction(36, 13, 0, robot.armToGrabAction()),
                grabAction(),
                robot.moveAndAction(-4, 30, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(36, 13, 0, robot.armToGrabAction()),
                grabAction(),
                robot.moveAndAction(-8, 30, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(54, 0, 180, 1.0, robot.lowerArmAction())
        );
        */

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double finishTime = 0.0;

        while (opModeIsActive()) {
            if (USE_VISION_LOCALIZATION) {
                robot.updateWithVision(telemetry);
            } else {
                robot.update(telemetry);
            }
            if (!robot.actionsDone())
                finishTime = timer.seconds();
            telemetry.addData("time", finishTime);
            telemetry.update();
        }

        Drivetrain.MAX_WHEEL_POWER = prevMaxWheelPower;
        PoseSingleton.getInstance().setPose(robot.dt.getPose());
    }
}
