package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous
public class CameraOpMode extends LinearOpMode {
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        List<MatOfPoint> contoursList = new ArrayList<>();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            contoursList.clear();

            Mat frame = new Mat();
            input.copyTo(frame);
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(frame, frame, 2);
            Imgproc.threshold(frame, frame, 102, 255, Imgproc.THRESH_BINARY_INV);
            List<MatOfPoint> tmpContours = new ArrayList<>();
            Imgproc.findContours(frame, tmpContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : tmpContours) {
                if (Imgproc.contourArea(contour) > 1000) {
                    contoursList.add(contour);
                }
            }

            /*MatOfPoint largest = null;
            double largestNum = 0;
            List<Integer> toRemove = new ArrayList<>();
            for (int i = 0; i < contoursList.size(); i++) {
                MatOfPoint contour = contoursList.get(i);
                double area = Imgproc.contourArea(contour);
                if (area > largestNum) {
                    largestNum = area;
                    largest = contour;
                }
                if (area < 100) {
                    contoursList.remove(i);
                }
            }

            for (int remove : toRemove) {
                if (remove >= 0 && remove < contoursList.size())
                    contoursList.remove(remove);
            }*/

            Imgproc.drawContours(input, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {

        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        waitForStart();

        while (opModeIsActive()) {
            sleep(100L);
        }
    }
}
