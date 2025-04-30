package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OpModes.Camera.AutoTakeSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class GetPositionSample {
    public static double initialAngle = Math.toRadians(15), h = 270.78 , cameraOffsetX = 100, cameraOffsetY = 140.148, centerToExtendo = 185; // COY = 140
    public static int MMToEncoderTicks(double distance){
        return (int)(distance / (2 * Math.PI * 16 / 4.75)) * 28;
    }

    public static Storage.SpecimenType getType(int type){
        if(type == 0) return Storage.SpecimenType.BLUE;
        if(type == 1) return  Storage.SpecimenType.RED;
        if(type == 2) return Storage.SpecimenType.YELLOW;
        return Storage.SpecimenType.NONE;
    }

    public static boolean hasId(LLResult res, int id){
        for(LLResultTypes.DetectorResult r : res.getDetectorResults()){
            if(r.getClassId() == id) return true;
        }
        return false;
    }

    public static double middleX = 630, middleY = -1600;

    @Deprecated
    public static LLResultTypes.DetectorResult getOptimalResultSmort(LLResult camera, int targetID){
        List<LLResultTypes.DetectorResult> detections = camera.getDetectorResults();

        // split the list into two separate lists
        List<LLResultTypes.DetectorResult> targetSamples = detections.stream().filter(e -> e.getClassId() == targetID).collect(Collectors.toList());
        detections = detections.stream().filter(e -> e.getClassId() != targetID).collect(Collectors.toList());
        for(LLResultTypes.DetectorResult detection : targetSamples){
            double distSampleRobot = getExtendoRotPair(detection.getTargetXDegreesNoCrosshair(), detection.getTargetYDegreesNoCrosshair()).x;
//            double distRobotToBar = Math.abs(XBarSub - Localizer.getCurrentPosition().x);
            double distRobotToBar = 30;
            double distRobotToSubBar = distRobotToBar / Math.cos(Localizer.getCurrentPosition().h);

            if(distSampleRobot >= distRobotToSubBar + 10 - centerToExtendo && // not close to a bar
                    distSampleRobot <= AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition() - 50) + centerToExtendo // not too far away
                    /*&& getPositionRelativeToRobot(detection.getTargetXDegrees(), detection.getTargetYDegrees()).y + Localizer.getCurrentPosition().y < halfYTerrain*/
                    //TODO: putini vecini aproape

            ){
                return detection;
            }
        }
        return targetSamples.get(0);
    }
    public static double XBarSub = 0, halfYTerrain = 0;
    public static LLResultTypes.DetectorResult getOptimalResult(LLResult result, int targetID){
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // split the list into two separate lists
        List<LLResultTypes.DetectorResult> targetSamples = detections.stream().filter(e -> e.getClassId() == targetID).collect(Collectors.toList());
        detections = detections.stream().filter(e -> e.getClassId() != targetID).collect(Collectors.toList());
//        double score[] = new double[targetSamples.size()];
        double min = 1e10;
        int id = 0;
//        for(LLResultTypes.DetectorResult detection : targetSamples){
        for(int i = 0; i < targetSamples.size(); i ++){
            LLResultTypes.DetectorResult detection = targetSamples.get(i);
            double distSampleRobot = getExtendoRotPair(detection.getTargetXDegreesNoCrosshair(), detection.getTargetYDegreesNoCrosshair()).x;
//            double distRobotToBar = Math.abs(XBarSub - Localizer.getCurrentPosition().x);
            double distRobotToBar = 30;
            double distRobotToSubBar = distRobotToBar / Math.cos(Localizer.getCurrentPosition().h);

            if(distSampleRobot <= AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition() - 50) - centerToExtendo){
                double lateralT = getPositionRelativeToRobot(detection.getTargetXDegrees(), detection.getTargetYDegrees()).y;
                double score = Math.sqrt(detection.getTargetXPixels() * detection.getTargetXPixels() + detection.getTargetYPixels() * detection.getTargetYPixels());
//                if(Localizer.getCurrentPosition().y + lateralT < middleY) score = 1e8;
                if(score < min){
                    id = i;
                    min = score;
                }
//                return detection;
            }
        }
        return targetSamples.get(id);
    }

    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(SparkFunOTOS.Pose2D fieldPos){
        double d = Localizer.getDistanceFromTwoPoints(fieldPos, Localizer.getCurrentPosition());
        double t = Localizer.getCurrentPosition().h - Math.atan2(fieldPos.y, fieldPos.x);
        double x = d * Math.cos(t);
        double y = d * Math.sin(t);
        return new SparkFunOTOS.Pose2D(x, y, 0);
    }

    public static SparkFunOTOS.Pose2D getExtendoRotPairByField(SparkFunOTOS.Pose2D s, SparkFunOTOS.Pose2D R){
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
        double fwd = pose.x - Extendo.MaxExtendoExtension;
        if(fwd < 0) fwd = 0;
        return new SparkFunOTOS.Pose2D(extendoDist, fwd, rot);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToFiled(double tx, double ty, SparkFunOTOS.Pose2D R){
        SparkFunOTOS.Pose2D p = getPositionRelativeToRobot(tx, ty);
        p.x *= -1;
        double d = Math.hypot(p.x, p.y);
        double t = Math.atan(p.y / p.x);
        double A = R.h + t - Math.PI;

        double x = d * Math.sin(A);
        double y = d * Math.cos(A);

        return new SparkFunOTOS.Pose2D(R.x + x, R.y + y, 0);
    }

}
