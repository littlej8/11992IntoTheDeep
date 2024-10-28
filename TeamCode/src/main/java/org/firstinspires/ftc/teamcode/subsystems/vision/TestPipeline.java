package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestPipeline extends OpenCvPipeline
{
    Mat mat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    Scalar ORANGE_MIN = new Scalar(5, 50, 50);
    Scalar ORANGE_MAX = new Scalar(17, 255, 255);

    Rect CENTER_ROI = new Rect(
        new Point(270, 220),
        new Point(360, 280)
    );

    Telemetry telemetry;

    public TestPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        contoursList.clear();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, ORANGE_MIN, ORANGE_MAX, mat);
        Mat kernel = new Mat();
        kernel.put(0, 0, 0);
        kernel.put(0, 1, 0);
        kernel.put(0, 0, 0);
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_HITMISS, kernel);

        Imgproc.findContours(mat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.drawContours(mat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);
        Core.inRange(mat, new Scalar(200, 200, 200), new Scalar(255, 255, 255), mat);

        Mat center = mat.submat(CENTER_ROI);

        contoursList.clear();
        Imgproc.findContours(center, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        center.release();

        double value = 0;
        for (MatOfPoint contour : contoursList)
            value += Imgproc.contourArea(contour);

        telemetry.addData("Value", value);
        telemetry.update();

        Scalar color;
        String text;
        if (value > 1000) {
            color = new Scalar(0, 255, 0, 255);
            text = "4 rings";
        } else if (value > 100) {
            color = new Scalar(0, 0, 255, 255);
            text = "1 ring";
        } else {
            color = new Scalar(255, 0, 0, 255);
            text = "no rings";
        }

        contoursList.clear();
        Imgproc.findContours(mat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contoursList, -1, new Scalar(0, 0, 255), 2, 8);

        Imgproc.rectangle(input, CENTER_ROI, color);
        Imgproc.putText(input, text, new Point(250, 210), 2, 1, new Scalar(255, 255, 255));

        return input;
    }

    /*Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound;

    enum Stage
    {
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
    }

    private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
    private Stage[] stages = Stage.values();

    private Telemetry telemetry;

    public StageSwitchingPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);
        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

        telemetry.addData("[Stage]", stageToRenderToViewport);
        telemetry.addData("[Found Contours]", "%d", numContoursFound);
        telemetry.update();

        switch (stageToRenderToViewport)
        {
            case YCbCr_CHAN2:
            {
                return yCbCrChan2Mat;
            }

            case THRESHOLD:
            {
                return thresholdMat;
            }

            case CONTOURS_OVERLAYED_ON_FRAME:
            {
                return contoursOnFrameMat;
            }

            case RAW_IMAGE:
            {
                return input;
            }

            default:
            {
                return input;
            }
        }
    }

    public int getNumContoursFound()
    {
        return numContoursFound;
    }*/
}
