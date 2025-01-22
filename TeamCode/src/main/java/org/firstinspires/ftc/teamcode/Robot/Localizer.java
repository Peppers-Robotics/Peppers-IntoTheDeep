package org.firstinspires.ftc.teamcode.Robot;

import android.net.IpSecManager;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.PinPoint;
import org.firstinspires.ftc.teamcode.OutTake.Elevator;

public class Localizer {

    public static double X = 132.5, Y = -0.6;
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.REVERSED, yPod = PinPoint.EncoderDirection.FORWARD;
    public static PinPoint pinPoint;
    private static Pose2D velocity = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private static Pose2D lastPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private static ElapsedTime time = new ElapsedTime();

    public static Pose2D Div(Pose2D pose, double d){
        return new Pose2D(DistanceUnit.MM, pose.getX(DistanceUnit.MM) / d, pose.getY(DistanceUnit.MM) / d, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES) / d);
    }
    public static void Initialize(HardwareMap hm){
        pinPoint = hm.get(PinPoint.class, "pinpoint");
        pinPoint.setOffsets(X, Y);
        pinPoint.setEncoderDirections(xPod, yPod);
        pinPoint.resetPosAndIMU();
    }

    public static void Update(){
        pinPoint.update();
        velocity = new Pose2D(DistanceUnit.MM,
                getCurrentPosition().getX(DistanceUnit.MM) - lastPose.getX(DistanceUnit.MM),
                getCurrentPosition().getY(DistanceUnit.MM) - lastPose.getY(DistanceUnit.MM), AngleUnit.DEGREES,
           getCurrentPosition().getHeading(AngleUnit.DEGREES) - lastPose.getHeading(AngleUnit.DEGREES));
        Div(velocity, time.seconds());
        time.reset();
    }
    public static void Reset(){
        pinPoint.recalibrateIMU();
        pinPoint.resetPosAndIMU();
    }
    public static void SetPose(Pose2D pose){
        pinPoint.setPosition(pose);
        Update();
    }
    public static Pose2D getCurrentPosition(){
        return pinPoint.getPosition();
    }
    /* PinPoint velocity is too noisy to be used for anything useful
        so a basic low pass filter won't be enough to make something useful.
        As for now we will do a basic velocity based on positions and time
        TODO: add kalman filter to reduce noise and raise accuracy
    */
    public static Pose2D getVelocity(){
        return velocity;
    }
}
