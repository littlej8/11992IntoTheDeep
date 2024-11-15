package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.runtime.Block;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

public class BlocksFunctions extends BlocksOpModeCompanion {
    public static Drivetrain dt;
    public static Arm arm;
    public static Slides slides;

    @ExportToBlocks(
            heading = "",
            tooltip = "Initializes the drivetrain"
    )
    /*@Block(
            methodName = "initDrivetrain",
            fieldName = ""
    )*/
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
        initDrivetrain();
        initArm();
        initSlides();
    }

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
    }
}