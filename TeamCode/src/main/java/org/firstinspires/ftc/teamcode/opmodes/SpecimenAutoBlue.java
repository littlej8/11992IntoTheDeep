package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Action;
import org.firstinspires.ftc.teamcode.subsystems.actions.ActionSequence;
import org.firstinspires.ftc.teamcode.subsystems.actions.MaintainSubsystemAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;
import org.firstinspires.ftc.teamcode.subsystems.actions.WaitAction;
import org.firstinspires.ftc.teamcode.util.PoseSingleton;

@Disabled
@Config
@Autonomous(name = "Specimen Auto", preselectTeleOp = "MainTeleOp")
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

        Arm.ADJUST_UP_5_IF_VOLTAGE_LOW = true;
        Arm.MAX_POWER = 1.0;
        //Drivetrain.MAX_WHEEL_POWER = MAX_SPEED;

        /*
        robot.schedule(
                robot.prepareToHookAction(),
                robot.moveAndAction(0, 20, 180, new MaintainSubsystemAction(robot.arm)),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 46, 180, new MaintainSubsystemAction(robot.arm)),
                robot.lowerArmAction(),
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
        );
        */

        robot.schedule(
                new ParallelAction(
                    robot.prepareToHookAction(),
                    new ActionSequence(
                        new WaitAction(1000),
                        new UntilAction(new WaitAction(2500), robot.moveAction(16, 10, 180))
                    )
                ),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 30, 180, new MaintainSubsystemAction(robot.arm)),
                robot.relocalizeAction(new Vector2d(7, 38)),
                new ParallelAction(
                    robot.lowerArmAction(),
                    new ActionSequence(
                        robot.moveAction(-48, 38, 180),
                        robot.turnToAction(0, 1500)
                    )
                ),
                robot.armToGrabSpecAction(),
                new UntilAction(
                        new WaitAction(3000),
                        robot.moveAndAction(-48, 64, 0, 0.2, new MaintainSubsystemAction(robot.arm))
                ),
                new UntilAction(
                        new WaitAction(1000),
                        new ParallelAction(
                            new Action() {
                            boolean init = false;
                            @Override
                            public boolean run(Telemetry telemetry) {
                                if (!init) {
                                    init = true;
                                    robot.dt.setTargetPose(new Pose2d(
                                            -48,
                                            robot.dt.getY() - 2,
                                            0
                                    ));
                                }

                                robot.dt.updatePose();
                                robot.dt.updateMovement();

                                if (robot.dt.moveFinished()) {
                                    robot.dt.killPowers();
                                    return false;
                                }

                                return true;
                            }
                        },
                            new MaintainSubsystemAction(robot.arm)
                        )
                ),
                new UntilAction(robot.claw.gripAction(), new MaintainSubsystemAction(robot.arm)),
                robot.prepareToHookAction(),
                robot.moveAndAction(-48, 50, 0, 0.4, new MaintainSubsystemAction(robot.arm)),
                new UntilAction(robot.turnToAction(180, 1500), new MaintainSubsystemAction(robot.arm)),
                robot.moveAndAction(20, 50, 180, new MaintainSubsystemAction(robot.arm)),
                new UntilAction(
                        new WaitAction(2500),
                        robot.moveAndAction(20, 10, 180, new MaintainSubsystemAction(robot.arm))
                ),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(20, 40, 180, new MaintainSubsystemAction(robot.arm)),
                robot.moveAndAction(-60, 64, 180, 1.0, robot.lowerArmAction())
                /*,
                new ParallelAction(
                    robot.lowerArmAction(),
                    new ActionSequence(
                        robot.moveAction(-44, 46, 180),
                        robot.moveAction(-44, 4, 180),
                        robot.turnToAction(-90),
                        robot.moveAction(-58, 4, -90),
                        robot.moveAction(-58, 56, -90)
                    )
                ),
                new ParallelAction(
                        robot.armToGrabSpecAction(),
                        new ActionSequence(
                                robot.moveAction(-55.5, 50, -90),
                                robot.turnToAction(0),
                                new UntilAction(new WaitAction(500), robot.maintainPositionAction())
                        )
                ),
                //robot.relocalizeAction(new Vector2d(-55.5, 50)),
                new UntilAction(new WaitAction(2000), robot.moveAndAction(-55.5, 60, 0, 0.3, new MaintainSubsystemAction(robot.arm))),
                robot.claw.gripAction(),
                new ActionSequence(
                    robot.moveAction(-55.5, 49, 0),
                    robot.prepareToHookAction(),
                    robot.moveAndAction(0, 30, 180, new MaintainSubsystemAction(robot.arm))
                ),
                new UntilAction(new WaitAction(2500), robot.moveAndAction(20, -10, 180, new MaintainSubsystemAction(robot.arm))),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 30, 180, new MaintainSubsystemAction(robot.arm)),
                new ParallelAction(
                    robot.lowerArmAction(),
                    robot.moveAction(-61, 64, 180)
                )
                //robot.relocalizeAction(new Vector2d(-55.5, 46)),
                /*new UntilAction(new WaitAction(1000), robot.moveAndAction(-61, 60, 0, 0.3, new MaintainSubsystemAction(robot.arm))),
                robot.claw.gripAction(),
                new ActionSequence(
                        robot.moveAction(-61, 58, 0),
                        robot.prepareToHookAction(),
                        robot.moveAndAction(0, 30, 180, new MaintainSubsystemAction(robot.arm))
                ),
                new UntilAction(new WaitAction(1500), robot.moveAndAction(4, 14, 180, new MaintainSubsystemAction(robot.arm))),
                robot.claw.dropAction(),
                robot.armAction(10),
                robot.moveAndAction(0, 30, 180, new MaintainSubsystemAction(robot.arm)),
                new ParallelAction(
                        robot.lowerArmAction(),
                        robot.moveAction(-64, 60, 180)
                )*/
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
