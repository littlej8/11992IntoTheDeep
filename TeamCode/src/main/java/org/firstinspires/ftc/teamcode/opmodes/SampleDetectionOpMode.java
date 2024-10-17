package org.firstinspires.ftc.teamcode.opmodes;

/*
 * Comment this when building in EOCV-sim
*/
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        waitForStart();

        while (opModeIsActive()) {
            sleep(100L);
        }
    }
}
