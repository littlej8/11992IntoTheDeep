package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.actions.UntilAction;

public class BlocksFunctions extends BlocksOpModeCompanion {
    public static Robot robot;
    public static Drivetrain dt;
    public static Arm arm;
    public static Slides slides;

    @ExportToBlocks(
            heading = "",
            tooltip = "Initializes the drivetrain"
    )
    public static void initDrivetrain() {
        dt = new Drivetrain(hardwareMap, telemetry);
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Initializes the arm"
    )
    public static void initArm() {
        arm = new Arm(hardwareMap, telemetry);
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Initializes the slides"
    )
    public static void initSlides() {
        slides = new Slides(hardwareMap, telemetry);
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Initializes the robot"
    )
    public static void initRobot() {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, Math.toRadians(-90)), true);
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Runs the actions previously scheduled in the robot"
    )
    public static void updateRobot() {
        robot.update();
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Schedules a moveTo for the robot",
            parameterLabels = {"x", "y", "heading"}
    )
    public static void scheduleMove(double x, double y, double h) {
        robot.schedule(robot.moveAction(x, y, h));
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Schedules a moveTo for the robot with a custom speed from 0-1",
            parameterLabels = {"x", "y", "heading", "speed"}
    )
    public static void scheduleMoveSpeed(double x, double y, double h, double speed) {
        robot.schedule(robot.moveAction(x, y, h, speed));
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Schedules a specimen grab for the robot",
            parameterLabels = {"x", "y", "heading"}
    )
    public static void scheduleGrabSpecimen() {
        robot.schedule(robot.claw.gripAction());
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Schedules a specimen hook for the robot",
            parameterLabels = {"x", "y", "heading"}
    )
    public static void scheduleHookSpecimen() {
        robot.schedule(robot.claw.bendAction(210));
    }

    @ExportToBlocks(
            heading = "",
            tooltip = "Schedules a move for the robot while moving the arm up",
            parameterLabels = {"x", "y", "heading", "speed"}
    )
    public static void scheduleMoveAndArmUp(double x, double y, double heading, double speed) {
        robot.schedule(new UntilAction(robot.moveAction(x, y, heading, speed), robot.prepareToHookAction()));
    }

    /*
    @ExportToBlocks(
            heading = "",
            comment = "Moves the robot to the specified position in inches relative to its starting position",
            tooltip = "Moves the robot to a position",
            parameterLabels = {"X Position", "Y Position", "Heading"}
    )
    public static void moveTo(double x, double y, double h) {
        dt.moveTo(x, y, h);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Holds the robot in the last position it was told to move to for the specified amount of milliseconds.",
            tooltip = "Maintains the robot's position for a length of time.",
            parameterLabels = "Time (Milliseconds)"
    )
    public static void maintainPosition(double millis) {
        dt.maintainPosition(millis);
    }*/
}
