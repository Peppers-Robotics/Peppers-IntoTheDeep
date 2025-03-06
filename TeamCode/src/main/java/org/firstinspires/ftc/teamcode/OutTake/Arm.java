package org.firstinspires.ftc.teamcode.OutTake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.DifferentialHelper;
import org.firstinspires.ftc.teamcode.HelperClasses.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.HelperClasses.Devices.ServoPlus;
import org.firstinspires.ftc.teamcode.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Config
public class Arm {
    public static double s1Offset = 345, s2Offset = 345;
    private static double armPrevPos = 0, pivotPrevPos = 0;
    public static ServoPlus servo1, servo2;
    public static AsymmetricMotionProfile armProfile, pivotProfile;
    private static final DifferentialHelper diffy;

    static {
        armProfile = new AsymmetricMotionProfile(8000, 4500, 3000);
        pivotProfile = new AsymmetricMotionProfile(3000, 4500, 2000);
        diffy = new DifferentialHelper(1);
    }


    public static void setArmAngle(double angle){
        angle *= -1;
        if(armProfile.getPosition() == angle) return;
        armProfile.startMotion(armPrevPos, angle);
        armPrevPos = angle;
    }
    public static double getArmTargetPosition(){
        return -armProfile.getTargetPosition();
    }

    public static void update(){
        armProfile.update();
        pivotProfile.update();
        diffy.setAngleToSecondJoint(armProfile.getPosition());
        diffy.setAngleToFirstJoint(pivotProfile.getPosition());

        servo1.setAngle(diffy.getRawAngles()[1] + s1Offset);
        servo2.setAngle(diffy.getRawAngles()[0] + s2Offset);

//        Robot.telemetry.addData("profiled arm angle", armProfile.getPosition());
//        Robot.telemetry.addData("profiled pivot angle", pivotProfile.getPosition());
//        Robot.telemetry.addData("Raw servo positions", "(" + servo1.getPosition() + ", " + servo2.getPosition() + ")");

    }

    @Deprecated
    public static void setPivotAngle(double angle){
//        angle = 0;
        if(pivotProfile.getPosition() == angle) return;
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
        return -armProfile.getPosition();
    }

}
