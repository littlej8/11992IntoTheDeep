package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import static java.lang.Math.toRadians;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23, -62, toRadians(90)))
                                .splineTo(new Vector2d(-48, -40), toRadians(90))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-55, -55, toRadians(45)))
                                .waitSeconds(3)
                                .splineTo(new Vector2d(-58, -40), toRadians(90))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-55, -55, toRadians(45)))
                                .waitSeconds(3)
                                .splineTo(new Vector2d(-68, -40), toRadians(90))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-55, -55, toRadians(45)))
                                .waitSeconds(3)
                                .splineTo(new Vector2d(0, -34), toRadians(90))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(-55, -55, toRadians(45)))
                                .waitSeconds(3)
                                .build()
                );

        Image img;
        img = ImageIO.read(new File("C:\\Users\\w260016\\Desktop\\robotics\\24-25repo\\MeepMeepTesting\\src\\main\\res\\field-2024-juice-dark.png"));
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}