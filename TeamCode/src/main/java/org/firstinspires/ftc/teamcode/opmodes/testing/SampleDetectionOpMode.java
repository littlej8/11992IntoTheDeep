package org.firstinspires.ftc.teamcode.opmodes.testing;

/*
 * Comment this when building in EOCV-sim
*/
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.vision.SampleDetectionVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

@Autonomous(name = "SampleDetectionOpMode")
public class SampleDetectionOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleDetectionVisionProcessor processor = new SampleDetectionVisionProcessor();

        new VisionPortal.Builder()
            .addProcessor(processor)
            .setCamera(BuiltinCameraDirection.BACK)
            .build();

        /*
        * Comment this when building in sim
        */
        FtcDashboard.getInstance().startCameraStream(processor, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            SampleDetectionVisionProcessor.DetectedSample largest = processor.getLargestDetection();
            if (largest != null) {
                double focalLength = 0.15748;
                double cameraCenterX = processor.getWidth() / 2.0;
                double cameraCenterY = processor.getHeight() / 2.0;
                double realObjectWidth = 1.5;
                double objectWidthInFrame = largest.rectFit.size.width;
                double objectXCoordinate = largest.rectFit.center.x;
                double objectYCoordinate = largest.rectFit.center.y;

                double distanceZ = (realObjectWidth * focalLength) / objectWidthInFrame;
                double distanceX = (distanceZ / focalLength) * (objectXCoordinate - cameraCenterX);
                double distanceY = (distanceZ / focalLength) * (objectYCoordinate - cameraCenterY);

                telemetry.addData("Largest Blob Screen Position", "(%d, %d)", objectXCoordinate, objectYCoordinate);
                telemetry.addData("Largest Blob Real Position (in)", "(%f, %f, %f)", distanceX, distanceY, distanceZ);
            }

            telemetry.update();
        }
    }
}
