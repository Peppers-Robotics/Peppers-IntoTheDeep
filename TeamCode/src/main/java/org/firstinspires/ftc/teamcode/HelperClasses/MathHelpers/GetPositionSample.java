package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OpModes.Camera.AutoTakeSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class GetPositionSample {
    public static double initialAngle = Math.toRadians(15), h = 270.78 , cameraOffsetX = 91.053, cameraOffsetY = 140.148, centerToExtendo = 162.664;
    public static int MMToEncoderTicks(double distance){
        return (int)(distance / (2 * Math.PI * 16 / 4.75)) * 28;
    }
    public static class Point2d{
        private double x, y;
        public Point2d(double xVal, double yVal){
            x = xVal;
            y = yVal;
        }
        public Point2d(){
            x = 0;
            y = 0;
        }

        public Point2d add(Point2d op){
            return new Point2d(x + op.x, y + op.y);
        }
        public Point2d sub(Point2d op){
            return new Point2d(x - op.x, y - op.y);
        }
        public Point2d mul(Point2d op){
            return new Point2d(x * op.x, y * op.y);
        }
        public Point2d div(Point2d op){
            return new Point2d(x / op.x, y / op.y);
        }

        public static double getSlope(Point2d A, Point2d B){
            return (A.y - B.y) / (A.x - B.x);
        }
        public static double getYintercept(Point2d A, Point2d B){
            return A.y - getSlope(A, B) * A.x;
        }
        public static boolean isPointInsideParallelLines(double m1, double m2, double n1, double n2, Point2d point){
            return m1 * point.x + n1 > point.y && point.y > m2 * point.x + n2;
        }
        public static double getDistance(Point2d A, Point2d B){
            return Math.sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
        }
    }

    public static double middleX = 630, middleY = -1000;

    public static LLResultTypes.DetectorResult getOptimalResultSmort(LLResult camera, int targetID){
        List<LLResultTypes.DetectorResult> detections = camera.getDetectorResults();

        // split the list into two separate lists
        List<LLResultTypes.DetectorResult> targetSamples = detections.stream().filter(e -> e.getClassId() == targetID).collect(Collectors.toList());
        detections = detections.stream().filter(e -> e.getClassId() != targetID).collect(Collectors.toList());

        double[] score = new double[targetSamples.size()];

        for(LLResultTypes.DetectorResult target : targetSamples){
        }
        int idx = 0;
        double mx = -1e10;
        for(int i = 0; i < score.length; i++){
            if(score[i] > mx){
                mx = score[i];
                idx = i;
            }
        }
        return targetSamples.get(idx);
    }
    public static double XBarSub = -470;
    public static LLResultTypes.DetectorResult getOptimalResult(LLResult result, int targetID){
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // split the list into two separate lists
        List<LLResultTypes.DetectorResult> targetSamples = detections.stream().filter(e -> e.getClassId() == targetID).collect(Collectors.toList());
        detections = detections.stream().filter(e -> e.getClassId() != targetID).collect(Collectors.toList());
        for(LLResultTypes.DetectorResult detection : detections){
            double distSampleRobot = getExtendoRotPair(detection.getTargetXDegreesNoCrosshair(), detection.getTargetYDegreesNoCrosshair()).x;
            double distRobotToBar = Math.abs(XBarSub - Localizer.getCurrentPosition().x);
            double distRobotToSubBar = distRobotToBar / Math.cos(Localizer.getCurrentPosition().h);

            if(distSampleRobot >= distRobotToSubBar + 10 - centerToExtendo){
                return detection;
            }
        }
        return detections.get(0);
    }

    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(SparkFunOTOS.Pose2D fieldPos){
        double d = Localizer.getDistanceFromTwoPoints(fieldPos, Localizer.getCurrentPosition());
        double t = Localizer.getCurrentPosition().h - Math.atan2(fieldPos.y, fieldPos.x);
        double x = d * Math.cos(t);
        double y = d * Math.sin(t);
        return new SparkFunOTOS.Pose2D(x, y, 0);
    }

    public static SparkFunOTOS.Pose2D getExtendoRotPairByField(SparkFunOTOS.Pose2D s){
        SparkFunOTOS.Pose2D R = Localizer.getCurrentPosition();
//        s = new SparkFunOTOS.Pose2D(R.x - s.x, R.y - s.y, 0);
//        double h = Math.atan2((s.y - R.y), (s.x - R.x));
        double r = Localizer.getDistanceFromTwoPoints(s, R);
        double h = Math.PI - Math.acos((s.x - R.x) / r);
        if(s.y - R.y < 0) h = Math.PI * 2 - h;

        h = Localizer.normalizeRadians(h);

        double e = Localizer.getDistanceFromTwoPoints(s, R) - centerToExtendo;
        double forward = e - AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition());
        if(forward < 0) forward = 0;
        return new SparkFunOTOS.Pose2D(e, forward, -h);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(double tx, double ty){
        double Y = h * Math.tan(Math.PI / 2 - initialAngle + Math.toRadians(ty));
        double X = Math.tan(Math.toRadians(tx)) * Y - cameraOffsetX;
        Y += cameraOffsetY;
        return new SparkFunOTOS.Pose2D(Y, X, 0);
    }
    public static SparkFunOTOS.Pose2D getExtendoRotPair(double tx, double ty){
        SparkFunOTOS.Pose2D pose = getPositionRelativeToRobot(tx, ty);
        double extendoDist = MMToEncoderTicks(Math.sqrt(pose.x * pose.x + pose.y * pose.y) - centerToExtendo);
        double rot = Math.atan(pose.y / -pose.x);
        return new SparkFunOTOS.Pose2D(extendoDist, 0, rot);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToFiled(double tx, double ty, SparkFunOTOS.Pose2D R){
        if(Localizer.pinPoint == null){
            RobotLog.e("Localizer not initialized, use `Localizer.Initialize(hardwareMap);`");
        }
        SparkFunOTOS.Pose2D p = getPositionRelativeToRobot(tx, ty);
        double omega = Math.PI + R.h - getExtendoRotPair(tx, ty).h;

        omega = Localizer.normalizeRadians(omega);

        double d = Math.sqrt(p.x * p.x + p.y * p.y);
        Robot.telemetry.addData("omega", Math.toDegrees(omega));

        double x = -R.x + d * Math.cos(omega);
        double y = -R.y - d * Math.sin(omega);

        return new SparkFunOTOS.Pose2D(x, y, 0);
    }

}
