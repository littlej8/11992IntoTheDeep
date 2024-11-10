package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class BlocksFunctions extends BlocksOpModeCompanion {
    public static Drivetrain dt;
    @ExportToBlocks(
            comment = "Initializes the drivetrain",
            tooltip = "Initializes the drivetrain"
    )
    public static void initDrivetrain() {
        dt = new Drivetrain(hardwareMap);
    }

    @ExportToBlocks(
            comment = "Moves the robot to the specified position in inches relative to its starting position",
            tooltip = "Moves the robot to a position",
            parameterLabels = {"X Position", "Y Position", "Heading"}
    )
    public static void moveTo(double x, double y, double h) {
        dt.moveTo(new Pose2d(x, y, Math.toRadians(h)));
    }
}
