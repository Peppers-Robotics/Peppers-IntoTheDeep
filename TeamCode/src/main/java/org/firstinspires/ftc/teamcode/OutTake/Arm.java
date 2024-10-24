package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.DifferentialHelper;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@Config
public class Arm {
    public static double s1Offset = 0, s2Offset = 0;
    public static ServoPlus servo1, servo2;
    private static AsymetricMotionProfile armProfile, pivotProfile;
    private static final DifferentialHelper diffy;

    static {
        armProfile = new AsymetricMotionProfile(0, 0, 0);
        pivotProfile = new AsymetricMotionProfile(0, 0, 0);
        diffy = new DifferentialHelper(1);
    }


    synchronized public static void setArmAngle(double angle){
        armProfile.startMotion(armProfile.getPosition(), angle);
    }

    synchronized public static void update(){
        armProfile.update();
        pivotProfile.update();
        diffy.setAngleToFirstJoint(armProfile.getPosition());
        diffy.setAngleToSecondJoint(pivotProfile.getPosition());

        servo1.setAngle(diffy.getRawAngles()[0]);
        servo2.setAngle(diffy.getRawAngles()[1]);
    }

    public synchronized static void setPivotAngle(double angle){
        pivotProfile.startMotion(pivotProfile.getPosition(), angle);
    }

    public static boolean motionCompleted(){
        return pivotProfile.motionEnded() && armProfile.motionEnded();
    }
    public static double getCurrentPivotAngle(){
        return pivotProfile.getPosition();
    }
    public static double getCurrentArmAngle(){
        return armProfile.getPosition();
    }

}
