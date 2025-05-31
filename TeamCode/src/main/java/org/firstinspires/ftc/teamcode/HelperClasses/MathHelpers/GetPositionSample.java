package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Autonomous.Sample;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Intake.Storage;
import org.firstinspires.ftc.teamcode.OpModes.Camera.AutoTakeSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.List;
import java.util.Vector;
import java.util.stream.Collectors;

@Config
public class GetPositionSample {
    public static double initialAngle = Math.toRadians(15), h = 270.78, centerToExtendo = 185;// COY = 140
    public static double cameraOffsetX = 140.148, cameraOffsetY = 100;
    public static double centerOffsetToDownRightX = 135.64,centerOffsetToDownRightY = 158.502;
    public static double CameraAngleFromCenterPoint = Math.atan(cameraOffsetY/cameraOffsetX);//de schimbat
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

    /* vlad */

    public static SparkFunOTOS.Pose2D getSampleRelativeToField(SparkFunOTOS.Pose2D cameraPos, double angleRobot, SparkFunOTOS.Pose2D samplePos) {
        double angleTotal = angleRobot - Math.tan(samplePos.x / samplePos.y);
        if(angleTotal < 0)
            angleTotal += 2*Math.PI;
        else if(angleTotal > Math.PI * 2)
            angleTotal -= Math.PI * 2;
        double ipo = Math.hypot(samplePos.x, samplePos.y);
        SparkFunOTOS.Pose2D translatedToCameraPos;

        if(angleTotal > 0 && angleTotal <= Math.PI/2)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(ipo*Math.cos(angleTotal),ipo*Math.sin(angleTotal),0);
        else if(angleTotal > Math.PI/2 && angleTotal <= Math.PI)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(-ipo*Math.sin(angleTotal-Math.PI/2),ipo*Math.cos(angleTotal-Math.PI/2),0);
        else if(angleTotal > Math.PI && angleTotal <= Math.PI*3/2)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(-ipo*Math.cos(angleTotal-Math.PI),-ipo*Math.sin(angleTotal-Math.PI),0);
        else
            translatedToCameraPos = new SparkFunOTOS.Pose2D(ipo*Math.sin(angleTotal-Math.PI*3/2),-ipo*Math.cos(angleTotal-Math.PI*3/2),0);

        return new SparkFunOTOS.Pose2D(/*centerOffsetToDownRightX +*/ translatedToCameraPos.x + cameraPos.x,/*centerOffsetToDownRightY +*/ translatedToCameraPos.y + cameraPos.y,0);
    }

    public static SparkFunOTOS.Pose2D getSamplePositionRelativeToCamera(double tx, double ty){

        double Y = h * Math.tan(Math.PI / 2 - initialAngle + Math.toRadians(ty));//doesnt work
        double X = Math.tan(Math.toRadians(tx)) * Y;
        return new SparkFunOTOS.Pose2D(X, Y, 0);
    }

    public static SparkFunOTOS.Pose2D normalize_pos(SparkFunOTOS.Pose2D posRobot){
        double normalizedDegrees = 0;
        if(Math.signum(posRobot.h) == 1)
            normalizedDegrees =  posRobot.h;
        else
            normalizedDegrees =  Math.PI*2 + posRobot.h;

        //to change if wrong
        double NaturalDisplacement = Math.PI/2;

        normalizedDegrees += NaturalDisplacement;
        if(normalizedDegrees > Math.PI*2)
            normalizedDegrees -= Math.PI*2;

        return new SparkFunOTOS.Pose2D(posRobot.y,-posRobot.x, normalizedDegrees) ;
    }

    public static SparkFunOTOS.Pose2D CameraRelativeToField(SparkFunOTOS.Pose2D positionRobot){

        SparkFunOTOS.Pose2D NormalizepositionRobot = normalize_pos(positionRobot);

        double z = Math.sqrt(cameraOffsetX*cameraOffsetX + cameraOffsetY*cameraOffsetY);

        double XcameraField = NormalizepositionRobot.x + -z * Math.sin((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));
        double YcameraField = NormalizepositionRobot.y + z * Math.cos((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));

        return new SparkFunOTOS.Pose2D(XcameraField,YcameraField,0);
    }

    public static SparkFunOTOS.Pose2D GetExtendoTicksToTravelAndNeededAngle(SparkFunOTOS.Pose2D RobotPos, SparkFunOTOS.Pose2D SampleFieldPos) {

        double delta_x = SampleFieldPos.x - RobotPos.x;
        double delta_y = SampleFieldPos.y - RobotPos.y;
        double angle = Math.atan2(delta_y,delta_x);
        if(angle < 0)
            angle = 2*Math.PI + angle;

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, RobotPos) - centerToExtendo;
        double forward = e - AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition());


        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,FromNormalizedToAbnormalAngle(angle));
    }

    private static double fromRadsToDegrees(double rads){
        return rads * 180 / Math.PI;
    }

    public static double FromNormalizedToAbnormalAngle(double angle){
        double startingAngles = Math.PI / 2; // Start of the positive interval
        //-pi -> pi

        if (angle >= 0 && angle <= startingAngles) {
            // Positive interval (from 0 to PI/2)
            return -(Math.PI/2 - angle);
        } else if (angle >= 3*Math.PI/2 && angle < 2*Math.PI) {
            // Positive interval (from PI/2 to PI)
            return -(Math.PI - (angle - Math.PI*3/2));
        } else{
            //pi/2 -> 2*pi/3
            return angle - startingAngles;
        }

    }

    public static int GetTicksDistanceFromTwoGlobals(SparkFunOTOS.Pose2D robotPos,SparkFunOTOS.Pose2D SamplePos){
        double distance = Math.sqrt((robotPos.x - SamplePos.x)*(robotPos.x - SamplePos.x) + (robotPos.y - SamplePos.y)*(robotPos.y - SamplePos.y));
        return MMToEncoderTicks(distance);
    }

    public static SparkFunOTOS.Pose2D CalculatePosFromMultipleScreenShots(Vector<SparkFunOTOS.Pose2D> Poses){

            SparkFunOTOS.Pose2D median = new SparkFunOTOS.Pose2D(0,0,0);
            for(SparkFunOTOS.Pose2D pos:Poses){
                median.x+=pos.x;
                median.y+=pos.y;
            }

            median.x /= Poses.size();
            median.y /= Poses.size();

            int ind = -1;
            double mini = 1e9;

            SparkFunOTOS.Pose2D best = new SparkFunOTOS.Pose2D(0,0,0);

            for(SparkFunOTOS.Pose2D pos:Poses){
                double temp = Localizer.getDistanceFromTwoPoints(pos,median);
                if(temp < mini)
                {
                    mini = temp;
                    best = pos;
                }
            }

        return best;
    }
}
