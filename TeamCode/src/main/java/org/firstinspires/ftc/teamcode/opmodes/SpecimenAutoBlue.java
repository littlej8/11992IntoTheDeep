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
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
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
                robot.moveAndAction(-41, 62, 0, robot.armToGrabSpecAction()),
                new UntilAction(
                    robot.claw.gripAction(),
                    robot.maintainPositionAction()
                )
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, new Pose2d(-8, 64, Math.toRadians(180)));
        robot.dt.setDriveRelativeToStart(false);
        double prevMaxWheelPower = Drivetrain.MAX_WHEEL_POWER;
        Drivetrain.MAX_WHEEL_POWER = MAX_SPEED;

        robot.schedule(
                robot.prepareToHookAction(),
                robot.moveAndAction(0, 20, 180, new MaintainSubsystemAction(robot.arm)),
                //robot.claw.bendAction(210),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 46, 180, new MaintainSubsystemAction(robot.arm)),
                robot.lowerArmAction(),
                //robot.moveAndAction(-40, 46, 180, new MaintainSubsystemAction(robot.arm)),
                robot.moveAction(-44, 46, 180),
                robot.moveAction(-44, 4, 180),
                robot.turnToAction(-90),
                robot.moveAction(-58, 4, -90),
                robot.moveAction(-58, 56, -90),
                robot.moveAction(-55.5, 50, -90),
                robot.turnToAction(0),
                robot.armToGrabSpecAction(),
                robot.moveAndAction(-55.5, 51.5, 0, 0.3, new MaintainSubsystemAction(robot.arm)),
                robot.claw.gripAction(),
                new ActionSequence(
                        robot.prepareToHookAction(),
                        robot.armAction(24),
                        robot.moveAndAction(0, 48, 180, new MaintainSubsystemAction(robot.arm))
                ),
                robot.moveAndAction(4, 15, 180, new MaintainSubsystemAction(robot.arm)),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 46, 180, new MaintainSubsystemAction(robot.arm)),
                robot.lowerArmAction(),
                robot.moveAction(-54, 50, 180)
                //new UntilAction(new WaitAction(2000), new MaintainSubsystemAction(robot.arm))
                /*robot.moveAndAction(-41, 51, 0, robot.armToGrabSpecAction()),
                robot.relocalizeAction(new Vector2d(-41, 48)),
                grabAction(),
                robot.moveAction(-41, 51, 0),
                robot.turnToAction(180),
                robot.moveAndAction(-6, 28, 180, robot.prepareToHookAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-41, 51, 180, robot.armToGrabAction()),
                robot.turnToAction(0),
                robot.relocalizeAction(new Vector2d(-38, 50)),
                grabAction(),
                robot.moveAction(-41, 51, 0),
                robot.turnToAction(180),
                robot.moveAndAction(-4, 28, 180, robot.prepareToHookAction()),
                new UntilAction(robot.hookSpecimenAction(), robot.maintainPositionAction()),
                robot.moveAndAction(-62, 64, 180, robot.lowerArmAction())*/
        );

        robot.claw.grip();

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
