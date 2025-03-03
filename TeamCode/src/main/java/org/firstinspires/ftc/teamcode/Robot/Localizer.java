package org.firstinspires.ftc.teamcode.Robot;

import android.net.IpSecManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.Devices.PinPoint;

@Config
public class Localizer {

    public static double X = 118.016, Y = 0.116, yawScalar = 1.0019;
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.FORWARD, yPod = PinPoint.EncoderDirection.FORWARD;
    public static PinPoint pinPoint;
    private static SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();
    private static SparkFunOTOS.Pose2D lastPose = new SparkFunOTOS.Pose2D();

    private static ElapsedTime time = new ElapsedTime();

    public static SparkFunOTOS.Pose2D Div(SparkFunOTOS.Pose2D pose, double d){
        return new SparkFunOTOS.Pose2D(pose.x / d, pose.y / d, pose.h / d);
    }

    public static double normalizeRadians(double raw){
        while(raw > Math.PI) raw -= Math.PI * 2;
        while(raw < -Math.PI) raw += Math.PI * 2;
        return raw;
    }

    public static void Initialize(HardwareMap hm){
        pinPoint = hm.get(PinPoint.class, "pinpoint");
        pinPoint.setOffsets(X, Y);
        pinPoint.setYawScalar(yawScalar);
        pinPoint.setEncoderDirections(xPod, yPod);
        pinPoint.resetPosAndIMU();
    }
    public static double getDistanceFromTwoPoints(SparkFunOTOS.Pose2D p1, SparkFunOTOS.Pose2D p2){
        if(p1 == null) p1 = new SparkFunOTOS.Pose2D();
        if(p2 == null) p2 = new SparkFunOTOS.Pose2D();
        return Math.sqrt(
                (p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y)
        );
    }
    public static double getAngleDifference(double h1, double h2){
        double d = Math.abs(h1 - h2);
        while(d < -Math.PI) d += 2 * Math.PI;
        while(d > Math.PI) d -= 2 * Math.PI;
        return d;
    }

    public static void Update(){
        pinPoint.update();
        velocity = new SparkFunOTOS.Pose2D(getCurrentPosition().x - lastPose.x, getCurrentPosition().y - lastPose.y, getCurrentPosition().h - lastPose.h);
        lastPose = getCurrentPosition();
        velocity = Div(velocity, time.seconds());

        Robot.telemetry.addData("pose", "(" + getCurrentPosition().x + ", " + getCurrentPosition().y + ", " + Math.toDegrees(getCurrentPosition().h) + "deg)");
        time.reset();
    }
    public static void Reset(){
        pinPoint.recalibrateIMU();
        pinPoint.resetPosAndIMU();
    }
    public static SparkFunOTOS.Pose2D getCurrentPosition(){
        double h = pinPoint.getHeading();
        h = normalizeRadians(h);
        return new SparkFunOTOS.Pose2D(pinPoint.getPosX(), pinPoint.getPosY(), h);
    }
    /* PinPoint velocity is too noisy to be used for anything useful
        so a basic low pass filter won't be enough to make something useful.
        As for now we will do a basic velocity based on positions and time
        TODO: add kalman filter to reduce noise and raise accuracy
    */
    public static SparkFunOTOS.Pose2D getVelocity(){
        return velocity;
    }
}
