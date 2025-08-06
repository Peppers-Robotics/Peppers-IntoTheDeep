package org.firstinspires.ftc.teamcode.HelperClasses.CameraDetection;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Config
public class YellowSampleDetectionPipeline extends OpenCvPipeline {
    private SparkFunOTOS.Pose2D poseWhenSnapshoted;
    public static boolean showMask = false;
    public static Size morphologicalKernel = new Size(3, 3),
                    erodeKernel = new Size(3, 3),
                    dilateKernel = new Size(3, 3);
    public static int erodeSteps = 1, dilateSteps = 2, morphologySteps = 2;
    public static Scalar lowerYellow = new Scalar(85, 30, 0), higherYellow = new Scalar(255, 255, 10);
    public static final double cameraFOV_X = 49.5, cameraFOV_Y = 60; // tune
    public static double SizeTreshold = 500;
    private Mat mask = new Mat(), tmp = new Mat(),
            labels = new Mat(), stats = new Mat(), centroids = new Mat();
    private double tx = 0, ty = 0;

    private static Point getMiddleTargetPoint(Mat stat, int i){
        double xM = (stat.get(i, Imgproc.CC_STAT_LEFT)[0] + stat.get(i, Imgproc.CC_STAT_WIDTH)[0]) / 2.d;
        double yM = (stat.get(i, Imgproc.CC_STAT_TOP)[0] + stat.get(i, Imgproc.CC_STAT_HEIGHT)[0]) / 2.d;

        return new Point(xM, yM);
    }

    private static Point getLowerTargetPoint(Mat stat, int i){
        double xM = (stat.get(i, Imgproc.CC_STAT_LEFT)[0] + stat.get(i, Imgproc.CC_STAT_WIDTH)[0]) / 2.d;
        double y = (stat.get(i, Imgproc.CC_STAT_TOP)[0] + stat.get(i, Imgproc.CC_STAT_HEIGHT)[0]);

        return new Point(xM, y);
    }

    @Override
    public Mat processFrame(Mat input) {
        poseWhenSnapshoted = Localizer.getCurrentPosition();
        double largestContour = -1;
//        Imgproc.cvtColor(input, BGRmap, Imgproc.COLOR_RGB2BGR);

        //make treshold
        Core.inRange(input, lowerYellow, higherYellow, tmp);

        //apply filters
        Imgproc.morphologyEx(tmp, mask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphologicalKernel), new Point(-1, -1), morphologySteps);
        tmp.release();
        Imgproc.erode(mask, tmp, Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, erodeKernel), new Point(-1, -1), erodeSteps);
        mask.release();
        Imgproc.dilate(tmp, mask, Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, dilateKernel), new Point(-1, -1), dilateSteps);
        tmp.release();


        Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8);
        double focalY = .5d * input.rows() / Math.tan(Math.toRadians(cameraFOV_Y / 2.d));
        double focalX = .5d * input.cols() / Math.tan(Math.toRadians(cameraFOV_X / 2.d));
        if(showMask){
            input = mask.clone();
        }

        for(int i = 0; i < centroids.rows(); i++){
            if(stats.get(i, Imgproc.CC_STAT_AREA)[0] < SizeTreshold) continue;

            // draw image for debugging

            double x = stats.get(i, Imgproc.CC_STAT_LEFT)[0],
                    y = stats.get(i, Imgproc.CC_STAT_TOP)[0],
                    w = stats.get(i, Imgproc.CC_STAT_WIDTH)[0],
                    h = stats.get(i, Imgproc.CC_STAT_HEIGHT)[0];

            //id
            Imgproc.putText(input, Integer.toString(i), new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            //bounding box
            Imgproc.drawContours(input, Arrays.asList(
                    new MatOfPoint(new Point(x, y)),
                    new MatOfPoint(new Point(x + w, y)),
                    new MatOfPoint(new Point(x + w, y + h)),
                    new MatOfPoint(new Point(x, y + h))

            ), -1, new Scalar(0, 255, 0), 2);


            // todo: add here field localization and other algorithms

            // get useful data
            if(stats.get(i, 0)[Imgproc.CC_STAT_AREA] <= largestContour) continue;
            largestContour = stats.get(i, 0)[Imgproc.CC_STAT_AREA];
            Point target = getLowerTargetPoint(stats, i);
            tx = Math.atan2((target.x - input.cols()) / 2.d, focalX);
            ty = Math.atan2((target.y - input.rows()) / 2.d, focalY);


        }
        mask.release();
        tmp.release();

        return input;
    }

    public double getTx(){
        return tx;
    }
    public double getTy(){
        return ty;
    }
    public SparkFunOTOS.Pose2D getPoseAtDetectionTime(){
        return poseWhenSnapshoted;
    }
}
