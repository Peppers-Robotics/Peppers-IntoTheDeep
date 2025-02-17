package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.OpModes.Camera.AutoTakeSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class GetPositionSample {
    public static double initialAngle = Math.toRadians(5), h = 286.2526 , cameraOffsetX = 105.053, cameraOffsetY = 110.67, centerToExtendo = 100;
    public static int MMToEncoderTicks(double distance){
        return (int)(distance / (2 * Math.PI * 16 / 4.75)) * 28;
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
        double h = Math.atan2(s.y - R.y, s.x - R.x);
        double e = Localizer.getDistanceFromTwoPoints(s, R) - centerToExtendo;
        return new SparkFunOTOS.Pose2D(e, e, h);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(double tx, double ty){
        double Y = h * Math.tan(Math.PI / 2 - initialAngle + Math.toRadians(ty));
        double X = Math.tan(Math.toRadians(tx)) * Y - cameraOffsetX;
        Y += cameraOffsetY;
        return new SparkFunOTOS.Pose2D(Y, X, 0);
    }
    public static SparkFunOTOS.Pose2D getExtendoRotPair(double tx, double ty){
        SparkFunOTOS.Pose2D pose = getPositionRelativeToRobot(tx, ty);
        double extendoDist = AutoTakeSample.DistanceToExtendo(pose.x - centerToExtendo);
        double rot = Math.atan(pose.y / pose.x);
        return new SparkFunOTOS.Pose2D(extendoDist, 0, rot * Math.signum(pose.y));
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToFiled(double tx, double ty, SparkFunOTOS.Pose2D R){
        if(Localizer.pinPoint == null){
            RobotLog.e("Localizer not initialized, use `Localizer.Initialize(hardwareMap);`");
        }
        SparkFunOTOS.Pose2D p = getPositionRelativeToRobot(tx, ty);
        double omega = R.h - getExtendoRotPair(tx, ty).h;

        double d = Math.sqrt(p.x * p.x + p.y * p.y);

        double x = R.x - d * Math.cos(omega);
        double y = R.y + d * Math.sin(omega);

        return new SparkFunOTOS.Pose2D(x, y, 0);
    }
}
