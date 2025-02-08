package org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.OpModes.Camera.AutoTakeSample;
import org.firstinspires.ftc.teamcode.Robot.Localizer;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class GetPositionSample {
    public static double initialAngle = Math.toRadians(25), h = 275.214, cameraOffsetX = 80, cameraOffsetY = 135, centerToExtendo = 200;
    public static int MMToEncoderTicks(double distance){
        return (int)(distance / (2 * Math.PI * 16 / 4.75)) * 28;
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
        double rot = Math.atan2(pose.y, pose.x);
        return new SparkFunOTOS.Pose2D(extendoDist, 0, -rot);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToFiled(double tx, double ty){
        if(Localizer.pinPoint == null){
            RobotLog.e("Localizer not initialized, use `Localizer.Initialize(hardwareMap);`");
        }
        SparkFunOTOS.Pose2D p = getPositionRelativeToRobot(tx, ty);
        SparkFunOTOS.Pose2D R = Localizer.getCurrentPosition();

        double d = Math.sqrt(p.x * p.x + p.y * p.y);
        double t = -R.h + Math.atan2(p.x, p.y);
        Robot.telemetry.addData("theta", Math.toDegrees(t));
        Robot.telemetry.addData("distance", d);
        double x = R.x + d * Math.sin(t);
        double y = R.y + d * Math.cos(t);

        return new SparkFunOTOS.Pose2D(x, y, 0);
    }
}
