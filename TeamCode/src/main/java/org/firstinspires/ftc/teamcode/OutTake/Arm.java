package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@Config
public class Arm {
    public static double s1Offset = 0, s2Offset = 0;
    public static ServoPlus servo1, servo2;
    public static AsymetricMotionProfile profile = new AsymetricMotionProfile(0, 0, 0);

    synchronized public static void setAngle(double angle){
        if(servo1.getAngle() == angle) return;
        profile.startMotion(servo1.getAngle(), angle);
    }

    synchronized public static void update(){
        profile.update();
        servo1.setAngle(profile.getPosition() + s1Offset);
        servo2.setAngle(profile.getPosition() + s2Offset);
    }

    public synchronized static void setRotation(){

    }

    public static boolean motionCompleted(){
        return profile.motionEnded();
    }

    public static double getPosition(){
        return profile.getPosition();
    }
}
