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
    public static double initialAngle = Math.toRadians(15), h = 270.78, centerToExtendo = 140;// COY = 140
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
            double distRobotToBar = 100;
            double distRobotToSubBar = distRobotToBar / Math.cos(Localizer.getCurrentPosition().h);

            if(distSampleRobot >= distRobotToSubBar + 10 - centerToExtendo && // not close to a bar
                    distSampleRobot <= AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition() - 100) + centerToExtendo // not too far away
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
        double max = -1;
        int id = 0;
//        for(LLResultTypes.DetectorResult detection : targetSamples){
        for(int i = 0; i < targetSamples.size(); i ++){
            LLResultTypes.DetectorResult detection = targetSamples.get(i);
            double distSampleRobot = getExtendoRotPair(detection.getTargetXDegreesNoCrosshair(), detection.getTargetYDegreesNoCrosshair()).x;
//            double distRobotToBar = Math.abs(XBarSub - Localizer.getCurrentPosition().x);
            double distRobotToBar = 200;
            double distRobotToSubBar = distRobotToBar / Math.cos(Localizer.getCurrentPosition().h);

            if(distSampleRobot <= AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition() - 50) - centerToExtendo){
//                double score = Math.sqrt(detection.getTargetXPixels() * detection.getTargetXPixels() + detection.getTargetYPixels() * detection.getTargetYPixels());
                double score = detection.getTargetArea();
//                if(Localizer.getCurrentPosition().y + lateralT < middleY) score = 1e8;
                if(score > max){
                    id = i;
                    max = score;
                }
//                return detection;
            }
        }
        if(targetSamples.isEmpty()) return null;
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

        double z = Math.hypot(cameraOffsetX,cameraOffsetY);

        double XcameraField = NormalizepositionRobot.x + -z * Math.sin((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));
        double YcameraField = NormalizepositionRobot.y + z * Math.cos((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));

        return new SparkFunOTOS.Pose2D(XcameraField,YcameraField,0);
    }

    public static SparkFunOTOS.Pose2D GetExtendoTicksToTravelAndNeededAngle(double tx,double ty) {

        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
        SparkFunOTOS.Pose2D normalizedPos = GetPositionSample.normalize_pos(pos);

        SparkFunOTOS.Pose2D poscamera = GetPositionSample.CameraRelativeToField(pos);

        SparkFunOTOS.Pose2D pos_sample = GetPositionSample.getSamplePositionRelativeToCamera(tx,ty);

        SparkFunOTOS.Pose2D SampleFieldPos = GetPositionSample.getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x) - Math.PI/2.f;
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition());
        if(forward < 0) forward = 0;


        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D GetExtendoTicksToTravelAndNeededAngleFromSample(SparkFunOTOS.Pose2D SampleFieldPos){

        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
        SparkFunOTOS.Pose2D normalizedPos = GetPositionSample.normalize_pos(pos);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x) - Math.PI/2.f;
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition());
        if(forward < 0) forward = 0;


        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D getContinuosTrackingData(SparkFunOTOS.Pose2D SampleFieldPos){
        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
        SparkFunOTOS.Pose2D normalizedPos = GetPositionSample.normalize_pos(pos);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x) - Math.PI/2.f;
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - AutoTakeSample.ExtendoToDistance(Extendo.getMaxPosition());
        if(forward < 0) forward = 0;

        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D GetGlobalSamplePosition(double tx,double ty){
        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
        SparkFunOTOS.Pose2D normalizedPos = GetPositionSample.normalize_pos(pos);

        SparkFunOTOS.Pose2D poscamera = GetPositionSample.CameraRelativeToField(pos);

        SparkFunOTOS.Pose2D pos_sample = GetPositionSample.getSamplePositionRelativeToCamera(tx,ty);

        return GetPositionSample.getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);

    }

    public static SparkFunOTOS.Pose2D CalculatePosFromMultipleScreenShots(Vector<SparkFunOTOS.Pose2D> Poses){

            SparkFunOTOS.Pose2D median = new SparkFunOTOS.Pose2D(0,0,0);
            for(SparkFunOTOS.Pose2D pos:Poses){
                median.x+=pos.x;
                median.y+=pos.y;
            }

            median.x /= Poses.size();
            median.y /= Poses.size();


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
