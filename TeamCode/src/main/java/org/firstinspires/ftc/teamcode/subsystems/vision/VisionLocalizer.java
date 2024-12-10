package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionLocalizer {
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -8, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private Pose2d latestPose = null;
    private double timeStamp = System.currentTimeMillis();

    public VisionLocalizer(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public boolean updateVisionPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.isEmpty())
            return false;

        List<Pose2d> poses = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null)
                continue;
            Position robotPose = detection.robotPose.getPosition();
            double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
            poses.add(new Pose2d(robotPose.x, robotPose.y, yaw));
        }

        if (poses.isEmpty())
            return false;

        double x = 0;
        double y = 0;
        double angle = 0;

        for (Pose2d pose : poses) {
            x += pose.position.x;
            y += pose.position.y;
            angle += pose.heading.toDouble();
        }

        latestPose = new Pose2d(x / poses.size(), y / poses.size(), angle / poses.size());
        timeStamp = System.currentTimeMillis();
        return true;
    }

    public Pose2d getLatestPose() {
        return latestPose;
    }

    public double getTimeStamp() {
        return timeStamp;
    }

    public boolean latestPoseValid() {
        return (System.currentTimeMillis() - timeStamp) < 100;
    }
}