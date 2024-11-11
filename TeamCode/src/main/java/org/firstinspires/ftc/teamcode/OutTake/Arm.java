package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.DifferentialHelper;
import org.firstinspires.ftc.teamcode.HelperClasses.ServoPlus;

@Config
public class Arm {
    public static double s1Offset = 180, s2Offset = 180;
    private static double armPrevPos = 0, pivotPrevPos = 0;
    public static ServoPlus servo1, servo2;
    public static AsymmetricMotionProfile armProfile, pivotProfile;
    private static final DifferentialHelper diffy;

    static {
        armProfile = new AsymmetricMotionProfile(4e3, 3e3, 1e4);
        pivotProfile = new AsymmetricMotionProfile(6e3, 6e3, 5e4);
        diffy = new DifferentialHelper(3/4.f);
    }


    public static void setArmAngle(double angle){
        if(armProfile.getPosition() == angle) return;
//        diffy.setAngleToSecondJoint(angle);
        armProfile.startMotion(armPrevPos, angle);
        armPrevPos = angle;
    }

    public static void update(){
        armProfile.update();
        pivotProfile.update();
        diffy.setAngleToSecondJoint(armProfile.getPosition());
        diffy.setAngleToFirstJoint(pivotProfile.getPosition());

        servo1.setAngle(diffy.getRawAngles()[1] + s1Offset);
        servo2.setAngle(diffy.getRawAngles()[0] + s2Offset);
    }

    public static void setPivotAngle(double angle){
        if(pivotProfile.getPosition() == angle) return;
//        diffy.setAngleToFirstJoint(angle);
        pivotProfile.startMotion(pivotPrevPos, angle);
        pivotPrevPos = angle;
    }

    public static boolean motionCompleted(){
        return pivotProfile.motionEnded() && armProfile.motionEnded();
    }
    public static double getCurrentPivotAngle(){
        return pivotProfile.getPosition();
    }
    public static double getPrecentOfArmMotionCompleted(){
        return armProfile.getPrecentOfMotion();
    }
    public static double getCurrentArmAngle(){
        return armProfile.getPosition();
    }

}
