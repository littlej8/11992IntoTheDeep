package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import android.annotation.SuppressLint;
import android.graphics.Canvas;

import java.util.concurrent.atomic.AtomicReference;
import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.opencv.android.Utils;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/*
 * Comment this when running on EOCV-sim
 */
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

/*
 * Also comment CameraStreamSource when running on sim 
 */
@Config
public class SampleDetectionVisionProcessor implements VisionProcessor, CameraStreamSource
{
    /*
     * The Bitmap to send to FtcDashboard
     */
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /*
     * Working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat hsvMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat redHighThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();
    Mat labeledImage = new Mat();

    /*
     * YCrCb Threshold values
     */
    // static final Scalar YELLOW_MASK_MIN = new Scalar(115, 155, 0);
    // static final Scalar YELLOW_MASK_MAX = new Scalar(255, 190, 78);
    // static final Scalar BLUE_MASK_MIN = new Scalar(0, 0, 150);
    // static final Scalar BLUE_MASK_MAX = new Scalar(255, 255, 255);
    //static final Scalar RED_MASK_MIN = new Scalar(0, 180, 86);
    //static final Scalar RED_MASK_MAX = new Scalar(255, 255, 255);

    /*
     * HSV Threshold values
     */
    public static Scalar YELLOW_MASK_MIN = new Scalar(14, 175, 75);
    public static Scalar YELLOW_MASK_MAX = new Scalar(51, 255, 255);
    public static Scalar BLUE_MASK_MIN = new Scalar(100, 75, 50);
    public static Scalar BLUE_MASK_MAX = new Scalar(150, 255, 255);
    public static Scalar RED_MASK_MIN = new Scalar(0, 100, 50);
    public static Scalar RED_MASK_MAX = new Scalar(6, 255, 255);
    public static Scalar RED_HIGH_MASK_MIN = new Scalar(160, 100, 50);
    public static Scalar RED_HIGH_MASK_MAX = new Scalar(180, 100, 50);

    /*
     * Min and Max Rectangle Area for detected samples
     */
    static final double RECT_MIN_SIZE = 250;
    static final double RECT_MAX_SIZE = 999999;

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2.5, 2.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2.5, 2.5));

    public static class DetectedSample
    {
        public RotatedRect rectFit;
        public double angle;
        public String color;
    }

    ArrayList<DetectedSample> internalSampleList = new ArrayList<>();
    volatile ArrayList<DetectedSample> clientSampleList = new ArrayList<>();

    int width = 0, height = 0;

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        internalSampleList.clear();

        input.copyTo(labeledImage);

        /*
         * Run the image processing
         */
        findContours(labeledImage);

        clientSampleList = new ArrayList<>(internalSampleList);

        labeledImage.copyTo(input);

        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);

        return null;
    }

    /*
     * Also comment this
     */
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public ArrayList<DetectedSample> getDetectedStones()
    {
        return clientSampleList;
    }
    public DetectedSample getLargestDetection() {
        if (clientSampleList.isEmpty())
            return null;

        DetectedSample largest = clientSampleList.get(0);
        for (DetectedSample sample : clientSampleList) {
            double largeArea = largest.rectFit.size.width * largest.rectFit.size.height;
            double thisArea = sample.rectFit.size.width * sample.rectFit.size.height;
            if (thisArea > largeArea)
                largest = sample;
        }

        return largest;
    }

    void findContours(Mat input)
    {
        // Convert the input image to YCrCb color space
        //Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, BLUE_MASK_MIN, BLUE_MASK_MAX, blueThresholdMat);
        Core.inRange(hsvMat, RED_MASK_MIN, RED_MASK_MAX, redThresholdMat);
        Core.inRange(hsvMat, RED_HIGH_MASK_MIN, RED_HIGH_MASK_MAX, redHighThresholdMat);
        Core.inRange(hsvMat, YELLOW_MASK_MIN, YELLOW_MASK_MAX, yellowThresholdMat);

        Core.bitwise_or(redThresholdMat, redHighThresholdMat, redThresholdMat);

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        filterContoursBySize(blueContoursList);
        filterContoursBySize(redContoursList);
        filterContoursBySize(yellowContoursList);

        // Create a plain image for drawing contours
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());

        // Analyze and draw contours
        for(MatOfPoint contour : blueContoursList)
        {
            analyzeContour(contour, input, "Blue");
        }

        for(MatOfPoint contour : redContoursList)
        {
            analyzeContour(contour, input, "Red");
        }

        for(MatOfPoint contour : yellowContoursList)
        {
            analyzeContour(contour, input, "Yellow");
        }
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply erosion and dilation for noise reduction
         */
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void filterContoursBySize(ArrayList<MatOfPoint> contours) {
        contours.removeIf(contour -> {
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(points);

            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            double area = rotatedRectFitToContour.size.width * rotatedRectFitToContour.size.height;
            return area < RECT_MIN_SIZE || area > RECT_MAX_SIZE;
        });
    }

    @SuppressLint("DefaultLocale")
    void analyzeContour(MatOfPoint contour, Mat input, String color)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Fit a rotated rectangle to the contour and draw it
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input, color);
        drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);

        // Adjust the angle based on rectangle dimensions
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
        {
            rotRectAngle += 90;
        }

        // Compute the angle and store it
        //double angle = -(rotRectAngle - 180);
        Rect r = Imgproc.boundingRect(contour2f);
        double focalLength = 822.317;
        double cx = 640.0 / 2.0;
        double cy = 480.0 / 2.0;
        double objYPix = r.height * 0.8;
        double objXPix = r.width;
        double zDist = (1.5 * focalLength) / objYPix;
        double xDist = (zDist / focalLength) * (objXPix - cx);
        double yDist = (zDist / focalLength) * (objYPix - cy);
        drawTagText(rotatedRectFitToContour, String.format("%.2f in", zDist), input, color);

        // Store the detected stone information
        DetectedSample detectedSample = new DetectedSample();
        detectedSample.rectFit = rotatedRectFitToContour;
        detectedSample.angle = 0;
        detectedSample.color = color;
        internalSampleList.add(detectedSample);
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color)
    {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                new Scalar(200, 200, 200), // Font color
                2); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        /*
         * Draws a rotated rectangle by drawing each of the 4 lines individually
         */
        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color)
    {
        switch (color)
        {
            case "Blue":
                return new Scalar(0, 0, 255);
            case "Yellow":
                return new Scalar(255, 255, 0);
            default:
                return new Scalar(255, 0, 0);
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
