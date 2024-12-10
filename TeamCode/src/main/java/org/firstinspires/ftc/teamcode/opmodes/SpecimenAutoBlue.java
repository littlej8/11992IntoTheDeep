package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Autonomous(name = "Specimen Auto Blue", preselectTeleOp = "MainTeleOp")
public class SpecimenAutoBlue extends LinearOpMode {
    Robot robot;

    // ram into the wall to reset position then grab
    private Action grabAction() {
        return new ActionSequence(
                robot.moveAndAction(44, 64, 0, 0.3, robot.armToGrabAction()),
                new UntilAction(
                    robot.grabSpecimenAction(),
                    robot.maintainPositionAction()
                )
        );
        /*return new ActionSequence(
                new UntilAction(
                        new WaitAction(500),
                        telemetry -> {
                            robot.dt.setDrivePowers(0, -0.5, 0);
                            robot.dt.updatePose(telemetry);
                            return false;
                        }
                ),
                new UntilAction(
                        robot.grabSpecimenAction(),
                        telemetry -> {
                            robot.dt.setDrivePowers(0, 0, 0);
                            robot.dt.setX(0);
                            return false;
                        }
                )
        );*/
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 64, Math.toRadians(180)), true);
        robot.dt.setDriveRelativeToStart(false);
        Drivetrain.MAX_WHEEL_POWER = 0.75;

        robot.schedule(
                robot.moveAndAction(0, 32, 180, 0.3, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(32, 46, 180, robot.lowerArmAction()),
                robot.moveAction(32, 4, 180),
                robot.moveAction(50, 4, -90),
                robot.moveAction(50, 56, -90, 0.4),
                //robot.relocalizeAction(new Vector2d(13, -52)),
                robot.moveAndAction(36, 51, 0, robot.armToGrabAction()),
                grabAction(),
                robot.moveAndAction(-12, 34, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(36, 51, 0, robot.armToGrabAction()),
                grabAction(),
                robot.moveAndAction(-16, 34, 180, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(54, 64, 180, 1.0, robot.lowerArmAction())
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

        /* fudged numbers that might work better

        robot.schedule(
                robot.moveAndAction(26, 0, -90, 0.3, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(18, -32, -90, robot.lowerArmAction()),
                robot.moveAction(48, -32, -90),
                robot.moveAction(48, -48, 0),
                robot.moveAction(3, -48, 0, 0.8),
                robot.moveAction(-6, -36, 90),
                new UntilAction(robot.grabSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(18, 4, -90, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-6, -36, 90, robot.lowerArmAction()),
                new UntilAction(robot.grabSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(17, 8, -90, robot.highBarAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-16, -48, -90, 1.0, robot.lowerArmAction())
        );*/

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double finishTime = 0.0;

        while (opModeIsActive()) {
            robot.updateWithVision(telemetry);
            if (!robot.actionsDone())
                finishTime = timer.seconds();
            telemetry.addData("time", finishTime);
            telemetry.update();
        }

        PoseSingleton.getInstance().setPose(robot.dt.getPose());
    }
}
