package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous
public class CameraOpMode extends LinearOpMode {
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        public static class Blob {
            private MatOfPoint contour;
            private Point[] contourPts;
            private int area = -1;
            private RotatedRect rect;

            Blob(MatOfPoint contour) {
                this.contour = contour;
            }

            public MatOfPoint getContour() {
                return this.contour;
            }

            public Point[] getContourPts() {
                if (this.contourPts == null) {
                    this.contourPts = this.contour.toArray();
                }

                return this.contourPts;
            }

            public int getContourArea() {
                if (this.area < 0) {
                    this.area = Math.max(1, (int)Imgproc.contourArea(this.contour));
                }

                return this.area;
            }

            public RotatedRect getBoxFit() {
                if (this.rect == null) {
                    this.rect = Imgproc.minAreaRect(new MatOfPoint2f(this.getContourPts()));
                }

                return this.rect;
            }

            public static void filterByArea(List<Blob> blobsToFilter, double minArea, double maxArea) {
                ArrayList<Blob> toRemove = new ArrayList<>();
                Iterator<Blob> it = blobsToFilter.iterator();

                while (true) {
                    Blob b;

                    do {
                        if (!it.hasNext()) {
                            blobsToFilter.removeAll(toRemove);
                            return;
                        }

                        b = it.next();
                    } while (!((double)b.getContourArea() > maxArea) && !((double)b.getContourArea() < minArea));

                    toRemove.add(b);
                }
            }
        }

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        public static Scalar[] BLUE_THRESH = {new Scalar(16.0, 0.0, 155.0), new Scalar(255.0, 127.0, 255.0)};
        public static Scalar[] RED_THRESH = {new Scalar(32.0, 176.0, 0.0), new Scalar(255.0, 255.0, 132.0)};
        public static Scalar[] YELLOW_THRESH = {new Scalar(32.0, 128.0, 0.0), new Scalar(255.0, 170.0, 120.0)};

        List<Blob> blobs = new ArrayList<>();
        double width = -1, height = -1;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            blobs.clear();

            Mat colorSpace = input.clone();
            Mat mask = new Mat();

            Imgproc.cvtColor(input, colorSpace, Imgproc.COLOR_RGB2YCrCb);
            Imgproc.GaussianBlur(colorSpace, colorSpace, new Size(5, 5), 0.0);

            Core.inRange(colorSpace, YELLOW_THRESH[0], YELLOW_THRESH[1], mask);

            List<MatOfPoint> contoursList = new ArrayList<>();
            Mat heirarchy = new Mat();
            Imgproc.findContours(mask, contoursList, heirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            heirarchy.release();

            for (MatOfPoint contour : contoursList) {
                blobs.add(new Blob(contour));
            }

            Blob.filterByArea(blobs, 50, 20000);

            for (Blob b : blobs) {
                Point[] contourPts = b.getContourPts();
                for (int i = 1; i < contourPts.length; i++) {
                    Point last = contourPts[i-1];
                    Point cur = contourPts[i];
                    Imgproc.line(input, last, cur, new Scalar(3, 227, 252), 4);
                }
            }

            if (width < 0)
                width = input.width();

            if (height < 0)
                height = input.height();

            Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(input, b);
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

        public double getScreenWidth() {
            return width;
        }

        public double getScreenHeight() {
            return height;
        }

        public Blob getLargestBlob() {
            if (blobs.isEmpty())
                return null;

            Blob largest = blobs.get(0);
            for (Blob b : blobs) {
                if (b.getContourArea() > largest.getContourArea())
                    largest = b;
            }

            return largest;
        }
    }
    @Override
    public void runOpMode() {
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            CameraStreamProcessor.Blob largestBlob = processor.getLargestBlob();

            if (largestBlob != null) {
                // real units are inches
                double focalLength = 0.15748;
                double cameraCenterX = processor.getScreenWidth() / 2;
                double cameraCenterY = processor.getScreenHeight() / 2;
                double realObjectWidth = 3.5;
                double objectWidthInFrame = largestBlob.getBoxFit().size.width;
                double objectXCoordinate = largestBlob.getBoxFit().center.x;
                double objectYCoordinate = largestBlob.getBoxFit().center.y;

                double distanceZ = (realObjectWidth * focalLength) / objectWidthInFrame;
                double distanceX = (distanceZ / focalLength) * (objectXCoordinate - cameraCenterX);
                double distanceY = (distanceZ / focalLength) * (objectYCoordinate - cameraCenterY);

                telemetry.addData("Largest Blob Screen Position", "(%d, %d)", largestBlob.getBoxFit().center.x, largestBlob.getBoxFit().center.y);
                telemetry.addData("Largest Blob Forward Dist", distanceZ);
                telemetry.addData("Largest Blob Sideways Dist", distanceX);
                telemetry.addData("Largest Blob Vertical Dist", distanceY);
            }

            telemetry.update();
            sleep(100L);
        }
    }
}
